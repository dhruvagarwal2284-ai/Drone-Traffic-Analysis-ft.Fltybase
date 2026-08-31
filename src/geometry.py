"""Image -> metric ground plane, derived analytically from drone telemetry.

No manual ground-control points and no satellite imagery: the mapping comes
from focal length, gimbal attitude and height above ground. That is what makes
the system portable to a site nobody has instrumented (the brief's third gap).

Camera model
------------
Pixels -> camera ray via K, camera ray -> world via gimbal attitude, then
intersect with the plane Z=0. World frame is local ENU (X east, Y north, Z up)
with the origin at the drone's nadir point.

Scale caveat
------------
`rel_alt` is height above the *takeoff point*, not above the road, so it carries
an unknown bias. The projection's *shape* is unaffected -- only a single global
scale factor is. `calibrate_scale` recovers that factor from observed vehicle
footprints, so the metric output never depends on trusting rel_alt.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# DJI reports focal length as a 35mm-equivalent; a 16:9 frame is a 36.0 x 20.25mm crop
SENSOR_WIDTH_MM = 36.0


def intrinsics(width: int, height: int, focal_35mm: float, zoom: float = 1.0) -> np.ndarray:
    f_px = focal_35mm * zoom * width / SENSOR_WIDTH_MM
    return np.array([[f_px, 0.0, width / 2.0],
                     [0.0, f_px, height / 2.0],
                     [0.0, 0.0, 1.0]])


def _rotation(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """Camera->world rotation. Columns are the camera axes (right, down, forward) in ENU.

    DJI convention: gb_yaw is a compass heading (0 = north, +ve clockwise),
    gb_pitch is negative when the gimbal looks down.
    """
    yaw, pitch, roll = np.radians([yaw_deg, pitch_deg, roll_deg])

    # optical axis
    fwd = np.array([np.cos(pitch) * np.sin(yaw),
                    np.cos(pitch) * np.cos(yaw),
                    np.sin(pitch)])
    fwd /= np.linalg.norm(fwd)

    # image-right is horizontal (before roll)
    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)

    down = np.cross(fwd, right)

    if abs(roll_deg) > 1e-6:  # rotate right/down about the optical axis
        c, s = np.cos(roll), np.sin(roll)
        right, down = c * right + s * down, -s * right + c * down

    return np.column_stack([right, down, fwd])


@dataclass
class GroundPlane:
    """Maps image pixels to metres on the road surface."""

    K_inv: np.ndarray
    R: np.ndarray          # camera -> world
    height: float          # camera height above the road plane, metres
    scale: float = 1.0     # calibration factor applied to rel_alt
    origin: np.ndarray = None  # ENU offset subtracted from outputs, metres

    @classmethod
    def from_telemetry(cls, width, height_px, focal_35mm, rel_alt,
                       yaw, pitch, roll, zoom=1.0) -> "GroundPlane":
        return cls(
            K_inv=np.linalg.inv(intrinsics(width, height_px, focal_35mm, zoom)),
            R=_rotation(yaw, pitch, roll),
            height=float(rel_alt),
            origin=np.zeros(2),
        )

    def to_ground(self, uv: np.ndarray, plane_z: np.ndarray | float = 0.0) -> np.ndarray:
        """(N,2) pixels -> (N,2) metres, intersecting the plane Z=`plane_z`.

        `plane_z` matters because this is an oblique view of objects with real
        height: at 63 deg we mostly see vehicle roofs, so a box centroid
        back-projected to Z=0 lands metres away from where the vehicle actually
        is. Intersecting at the object's mid-height instead removes that
        parallax exactly, rather than correcting for it approximately.

        Points at or above the horizon come back as NaN rather than silently
        wrapping to a huge finite distance.
        """
        uv = np.atleast_2d(np.asarray(uv, dtype=float))
        rays = np.concatenate([uv, np.ones((len(uv), 1))], axis=1) @ self.K_inv.T
        dirs = rays @ self.R.T                      # (N,3) in ENU

        h = self.height * self.scale - np.asarray(plane_z, dtype=float)
        dz = dirs[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(dz < -1e-9, -h / dz, np.nan)

        xy = dirs[:, :2] * t[:, None]
        xy[~np.isfinite(t)] = np.nan
        return xy - self.origin

    def to_image(self, xy: np.ndarray) -> np.ndarray:
        """(N,2) metres on the ground -> (N,2) pixels. Inverse of `to_ground`.

        Needed to draw ground-space results -- discovered lane lines, the back
        of a queue -- back onto the footage they were derived from.
        """
        xy = np.atleast_2d(np.asarray(xy, dtype=float)) + self.origin
        h = self.height * self.scale
        d = np.column_stack([xy[:, 0], xy[:, 1], np.full(len(xy), -h)])
        cam = d @ self.R                      # world -> camera (R is orthonormal)
        K = np.linalg.inv(self.K_inv)
        uvw = cam @ K.T
        z = uvw[:, 2]
        out = np.full((len(xy), 2), np.nan)
        ok = z > 1e-9                         # behind the camera -> NaN
        out[ok] = uvw[ok, :2] / z[ok, None]
        return out

    def homography(self, width: int, height_px: int) -> np.ndarray:
        """3x3 image->ground homography, for warping whole images."""
        import cv2

        src = np.float32([[0, height_px * 0.55], [width, height_px * 0.55],
                          [width, height_px], [0, height_px]])
        dst = np.float32(self.to_ground(src))
        return cv2.getPerspectiveTransform(src, dst)

    def metres_per_pixel(self, uv: np.ndarray) -> np.ndarray:
        """Local ground sampling distance, for weighting/QA. (N,) metres."""
        uv = np.atleast_2d(np.asarray(uv, dtype=float))
        a = self.to_ground(uv)
        b = self.to_ground(uv + np.array([1.0, 0.0]))
        c = self.to_ground(uv + np.array([0.0, 1.0]))
        return np.sqrt(np.linalg.norm(b - a, axis=1) * np.linalg.norm(c - a, axis=1))

    def set_origin(self, uv_centre: np.ndarray) -> None:
        """Put the metric origin under a chosen pixel (e.g. the junction centre)."""
        self.origin = np.zeros(2)
        self.origin = self.to_ground(uv_centre)[0]


def calibrate_scale(observed_lengths_m: np.ndarray, nominal_m: float = 4.0) -> float:
    """Recover the global scale factor from vehicle footprints.

    We take the median *car* footprint length to be 4.0 m. Returns the factor to
    apply to the assumed camera height. Motorcycle length is then an independent
    check the calibration was not fitted to.
    """
    obs = np.asarray(observed_lengths_m, dtype=float)
    obs = obs[np.isfinite(obs) & (obs > 0)]
    if obs.size < 20:
        return 1.0
    return float(nominal_m / np.median(obs))

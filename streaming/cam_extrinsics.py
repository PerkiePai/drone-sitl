"""The down_cam <-> body extrinsic, derived rather than measured.

vio-streamer.py cannot read a recorded cam_calib.json -- it has no dataset --
so it computes the extrinsic from the stage's authored geometry. That
derivation lives here, importable outside Kit, so it can be pinned against a
real recorded calibration instead of only being checked live at startup.

See test_cam_extrinsics.py: the analytic value agrees with the recorder's
measured extrinsic to ~0.01 deg.
"""
import numpy as np
from scipy.spatial.transform import Rotation

FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])
"""FLU<->FRD, self-inverse. 180 deg about x."""


def analytic_R_body_cam(img_roll_deg):
    """body(FLU) -> camera rotation for the hard-mounted down_cam.

    /World/down_mount carries the body's attitude composed with a constant
    DOWN_IMG_ROLL_DEG roll about the optical axis
    (drone_setup_px4_cesium.py:291), and down_cam sits under it with an
    identity local transform (drone_setup_px4_cesium.py:226) -- so the camera
    axes ARE the mount axes and the whole extrinsic is that one roll.

    Valid only with DOWN_VIB_DAMP=False. With the soft mount on, the mount
    orientation is a low-passed copy of the body's, which makes this extrinsic
    time-varying and the rig no longer VIO-valid (design D3).
    """
    return Rotation.from_euler("XYZ", [0.0, 0.0, img_roll_deg],
                               degrees=True).as_matrix()


def to_pipeline_R_CtoI(R_body_cam):
    """Apply load_dataset's convention fix so streaming and batch agree.

    flow_odometry.load_dataset:75 post-multiplies the recorded extrinsic by
    diag(1,-1,-1): the extrinsic is authored against the FRD IMU body while the
    GT attitudes are FLU-in-ENU, and the two differ by 180 deg about x. Doing
    the same here is what makes a streamed R_CtoI interchangeable with a
    recorded one -- if this drifts, the estimator silently mis-rotates every
    flow vector, which looks like a plausible-but-wrong trajectory rather than
    an error.
    """
    return np.asarray(R_body_cam, dtype=float) @ FLU_TO_FRD


def rot_angle_deg(A, B):
    """Angle of the rotation taking A to B, in degrees."""
    c = (np.trace(np.asarray(A).T @ np.asarray(B)) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))

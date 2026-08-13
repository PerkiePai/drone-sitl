# The estimator gets a heading reference: magnetometer now, visual yaw next

The 2026-08-07 design justified fusing vision yaw into EKF2 with "yaw, which has
no compass aiding". That premise is wrong in one direction and unguarded in the
other. **EKF2 has a compass**: Pegasus sends PX4 magnetometer data
(`px4_mavlink_backend.py:422-435,770`) and `EKF2_MAG_TYPE` defaults to auto, so
PX4 is not blind to heading with GNSS cut. **The estimator does not**:
`vio-streamer.py` reads only IMU and barometer, the `imu` topic carries `w` and
`a`, and `pipeline-streaming.py:80` calls `state.update(w, a, dt)` with `mag=None`
every tick — so live vision yaw is integrated gyro, unobservable about the
vertical and free to walk, even though `MahonyState.update()` and
`calibrate_mag()` already support a magnetometer and the batch path exercises
them.

We wire the magnetometer through the `imu` topic now, and queue **visual yaw** —
in-plane rotation estimated from the flow field — as the real fix. A magnetometer
is an onboard sensor and not a GNSS one, so it is inside the definition of VIO
this project uses (`CONTEXT.md`): the exclusion is GNSS, not everything other
than camera and IMU.

## Consequences

Yaw drift corrupts more than the yaw EKF2 fuses: flow-odom integrates translation
into ENU through the Mahony attitude, so a rotating heading rotates the position
stream itself. Turning off EV yaw fusion would not have addressed that, which is
why the fix is at the source.

For the classification flight the magnetometer is carried at **zero gain**
(ADR-0002): the data is recorded so offline analysis can show what it would have
corrected, and enabling it afterwards is one flag.

Visual yaw is the transferable answer. Flow-odom currently takes rotation as an
*input* (`R_c1c0` from the AHRS, `pipeline.py:242-247`) and never estimates it
from the imagery, discarding the one heading source that survives both GNSS
denial and a disturbed magnetic environment. Note also that the batch pipeline's
`--attitude ahrs_compass` mode is a **ground-truth heading proxy**
(`flow_odometry.py:239-253`), not a compass, so any offline accuracy quoted from
it was never a demonstration of unaided flow-odom.

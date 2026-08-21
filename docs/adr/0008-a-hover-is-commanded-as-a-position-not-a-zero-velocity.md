# A hover is commanded as a position, not as a zero velocity

Every station-keeping run this project has flown -- run 6's 225 m excursion, the
whole Task 10 `MPC_XY_*` screening campaign, `low_integral`'s altitude-drop abort
-- commanded the hold as `send_velocity(0, 0, 0)` in `MAV_FRAME_BODY_NED`. Read
back off the flight logs, `offboard_control_mode.position` is `0` and
`trajectory_setpoint.position` is `NaN` for **100%** of every GPS-denied hold on
record, with `velocity` pinned at exactly `(0, 0, 0)`.

That is an open integrator in position. PX4's position controller never computes
a position error, so `MPC_XY_P` is not in the loop at all and nothing acts on
displacement: whatever bias the estimator carries in its **velocity** walks the
airframe away, and no term pulls it back. In `logs/20260821-fix/11_16_00.ulg`
EKF2's own `vehicle_local_position` reports the aircraft 54 m from the cut point
by the end of the hold -- the estimate saw the excursion the entire time, and
there was no setpoint that could have responded to it.

Under GNSS the velocity bias is small enough that the resulting drift reads as
station keeping, which is why this survived every GPS baseline flight.

Idle in OFFBOARD now sends a **position** setpoint in `MAV_FRAME_LOCAL_NED`,
captured at the moment the last held direction is released.

## Consequences

The hover is a closed loop for the first time, so `MPC_XY_P` and `MPC_Z_P` now
have an effect they did not have before. **Task 10's gain sweep results are void
as guidance**: it ranked candidates on a position-loop gain that was not in the
loop, and its one apparent signal (`MPC_XY_VEL_I_ACC` low scoring best) is
explained by the velocity integrator being the only thing that could fight a
velocity bias when nothing else was closed. Any re-tune starts from PX4 defaults.

The hold point is captured once and frozen. Re-taking it from the aircraft each
tick would hold the position error at zero by construction -- the same open loop
wearing a position setpoint's shape.

It tracks the airframe whenever PX4 is not in OFFBOARD, because PX4 discards
setpoints outside OFFBOARD (`mavlink_receiver.cpp:1163`) and a point latched on
the pad would otherwise survive a 50 m `AUTO.TAKEOFF` and command a dive back to
the ground the instant OFFBOARD engaged.

It is dropped at the cut and at the restore. EKF2 resets its own horizontal
position when GNSS fusion changes (`ev_pos_control.cpp:182-233`); the airframe
does not move for it, but the frame the hold point is expressed in does, so a
point carried across that jump would command the reset distance as a real flight.

Without a `LOCAL_POSITION_NED` yet there is no point to hold, and it falls back
to the zero velocity. A gap in the stream drops OFFBOARD, so there is no third
option where nothing is sent.

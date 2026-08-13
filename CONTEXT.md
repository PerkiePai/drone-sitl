# GPS-denied flight (drone-sitl)

Flying the SITL drone positioned by this repo's own flow-odometry instead of the
simulator's GNSS, commanded from the web UI. This glossary exists because the
run write-ups in `SESSION.md` used one word — "drift" — for three different
quantities, and because the flight profile has four states that were being
called "phases" without agreed numbers.

## Language

**VIO**:
Positioning from the camera and the IMU together with the aircraft's other
onboard sensors — barometer, magnetometer — and **no GNSS**. The exclusion is
GNSS specifically, not "everything but camera and IMU": any sensor the airframe
carries is in scope, and ground truth never is.
_Avoid_: visual odometry (that is the camera alone), vision-only

### Flight profile

**Phase 0**:
The pre-flight state in which PX4 is given every parameter it only re-reads at
boot, and is then restarted. Nothing else is sent while it is held.
_Avoid_: boot params, reboot step

**Phase 1**:
Ordinary GNSS flight with vision not fused, asserted explicitly at every startup
rather than assumed from the previous run.
_Avoid_: baseline, normal flight

**Phase 1b**:
GNSS and vision both fused. The state in which the vision source is proven in
flight while a trusted position source is still present.
_Avoid_: dual mode, blended

**Phase 2**:
GNSS fusion off; the aircraft is positioned by vision alone.
_Avoid_: GPS-denied mode (the *system* is GPS-denied; phase 2 is the state)

**The cut**:
The transition from phase 1b to phase 2. Refusable, because taking it onto an
unproven source can put the aircraft in the ground.
_Avoid_: handover (that is the whole 1→1b→2 sequence), switchover

**The restore**:
The transition from phase 2 back to phase 1b. Takes no preconditions and cannot
refuse, because a recovery control that can say no is not one.
_Avoid_: abort, revert

**Handover**:
The whole sequence by which positioning responsibility moves from GNSS to
vision — phase 1 through phase 2, including both realignments.

### Frames and error

**Vision frame**:
The estimator's own ENU frame, originating where the estimator started and free
to rotate and translate away from the world as the estimate drifts.

**Realignment**:
The rigid transform that pins the vision frame onto PX4's local frame, solved at
a phase transition and then held fixed. Never re-solved continuously — a
continuously re-solved alignment would feed PX4 its own estimate back as an
independent measurement.
_Avoid_: snap, reset, correction

**Estimator drift**:
The distance between the estimator's reported position and ground truth. A
property of the estimator alone; it never sees any realignment.
_Avoid_: drift (unqualified), error

**Aircraft excursion**:
The distance between where the airframe *truly* is and where it was commanded to
hold. The quantity the hover bar is judged on, and independent of what any
estimator believes.
_Avoid_: drift (unqualified), position error

**Alignment offset**:
How far a realignment shifted the vision frame — the error that would otherwise
have been inherited at that transition.

### Time

**Sim time**:
The simulator's clock, which the flight controller also runs on under lockstep.
Every duration describing the aircraft's behaviour belongs on this clock.
_Avoid_: PX4 time, boot time

**Wall time**:
The operator's clock. Belongs only to durations describing the operator or the
host machine, never to anything the airframe experiences.
_Avoid_: real time

**Sim rate**:
Sim seconds elapsed per wall second. It wanders with host load, which is why any
quantity mixing the two clocks is not a constant.

### Sensing

**Blind nadir**:
The condition in which the downward camera sees no usable texture, so no
position can be solved. At this site it is a property of low altitude, and it is
why the handover cannot begin on the pad.

**Heading reference**:
An absolute yaw source. Distinct from integrated gyro yaw, which is
unobservable about the vertical and walks without one.

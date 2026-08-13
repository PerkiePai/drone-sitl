# Classify the failure mode before tuning any EKF2 parameter

Five live runs each spent one flight testing one hypothesis, and `SESSION.md`
closed run 5 by naming `EKF2_EV_DELAY` as the next suspect. The next flight
changes no parameter at all: it records the x/y track of PX4's estimate, the raw
estimator and ground truth, plus vision yaw against true yaw, and the shape of
the motion decides what to fix. An orbit or spiral indicts heading; a
straight-line back-and-forth indicts lag or loop gain.

The reasoning that forced this: the observed mode has a 40–60 s period, while the
whole camera→estimator→ZMQ→server→MAVLink path is a few hundred milliseconds.
Phase lag of that size perturbs a position loop at its own bandwidth — periods of
seconds. Blaming it would have repeated the run-2 mistake, where the one
unverified quantity attracted the blame precisely because everything else looked
healthy.

## Consequences

One flight now answers several hypotheses instead of one, because it is recorded
(ADR-0003) rather than watched. Nothing that changes flight behaviour may ride
along with it — which is why the magnetometer is wired but left at zero gain
(ADR-0005).

# Ground truth crosses into the MAVLink-owning process, for scoring only

Design decision D4 kept ground truth on its own topic, reaching the estimator and
stopping there, so it could never enter the estimate. That left nothing able to
compute the quantity 8.6 is actually judged on: `drift_m` is the estimator's error
against truth, and PX4's local position is the estimate under test — scoring it
against itself would show a rock-steady hover while the aircraft flew away. The
estimator therefore forwards ground truth alongside its estimate, and
`joystick-server` computes aircraft excursion from the hold point, shows it on the
VIO row and writes it to the run CSV.

## Consequences

Ground truth now exists inside the process that owns the MAVLink link, and an
accidental leak into the control path would be indistinguishable from success. It
is fenced by a test asserting `VisionPositionSender` never reads the field — the
same shape of guarantee D4 originally got from physical separation, which this
gives up deliberately in exchange for seeing the envelope grow during the flight
rather than after landing.

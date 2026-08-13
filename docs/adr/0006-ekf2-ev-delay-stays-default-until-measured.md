# EKF2_EV_DELAY stays at its default until the applied delay is measured

Under lockstep PX4's `hrt_absolute_time()` returns the lockstep scheduler's clock
(`drv_hrt.cpp:101-105`), so PX4 runs on **sim time**; and because timesync never
converges, every VISION_POSITION_ESTIMATE is stamped with PX4's own arrival time.
`EKF2_EV_DELAY` is therefore expressed in sim milliseconds — while much of the
real pipeline latency (JPEG encode, ZMQ, LK solve, the 20 Hz tick) is wall-clock
CPU work that maps into sim time scaled by `sim_rate`, observed wandering
0.41–0.59. The rest of the lag (frame interval, render sync) is already sim-timed
and does not scale. So the true delay is a mixture, only partly rate-dependent,
and **no single value is correct for a sim whose rate moves**.

We leave it at its default and extract the applied delay against `sim_rate` from
the classification flight's ulog (`estimator_aid_src_ev_pos` carries the fusion
timestamp against the sample timestamp) before deciding whether to set it at all.

## Consequences

This is the fourth wall-versus-sim-time question in this system, after
`PX4_RESTART_GAP_S`, the VPE timestamp theory and the staleness clock. Recorded
so the next reader does not "fix" the default without measuring — and so that if
the delay does track `sim_rate`, the conclusion is to stabilise the sim rate
rather than to tune a parameter against the host's load. Note `EKF2_EV_DELAY` is
`@reboot_required` (`ekf2_params.c:139-151`), so setting it means adding it to
phase 0, not to the fusion params.

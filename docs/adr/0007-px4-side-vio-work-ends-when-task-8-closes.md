# PX4-side VIO work ends when Task 8 closes

The EKF2 parameter set, the phase machine, the reboot ordering and the vision
bridge are PX4-specific and do not transfer to the flight controller this work is
ultimately aimed at. What transfers is flow-odometry itself, the phased-handover
*concept*, the frame alignment, and the instrumentation built in ADR-0003/0004.
So: pass 8.6, then run 8.7–8.9 (manual flight, waypoint route, estimator-kill
failsafe) in the same session while the rig is hot, close Task 8, and move to
visual yaw and estimator accuracy.

The reason for sweeping 8.7–8.9 immediately rather than dropping them is that they
are the tasks that test *flying* rather than hovering, they are cheap once a hover
holds, and every one of them has so far been blocked behind a configuration that
took five runs to reach.

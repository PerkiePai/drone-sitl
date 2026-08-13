# Flight logs are copied out of Pegasus's temp rootfs by a repo script

Pegasus launches PX4 with `cwd=tempfile.TemporaryDirectory()`
(`px4_launch_tool.py:44,63`), a directory destroyed when Isaac exits. The newest
ulog on this machine predates every VIO run: five flights of live debugging left
no flight recorder data at all. `sim/save-ulog.sh` resolves PX4's working
directory through `/proc/<pid>/cwd` while the sim is still up and copies the logs
into the repo, and `SDLOG_MODE=2` / `SDLOG_PROFILE=131` (default + EKF2-replay +
computer-vision topics) are set in phase 0, where the existing reboot makes both
`@reboot_required` params take effect for free.

## Considered Options

Patching Pegasus's launch tool to use a durable rootfs is one line, but it lives
outside this repo, vanishes on update, and would also make PX4's **parameter
file** persist across Isaac restarts. That converts "params persist across runs"
from something probably only ever true within one Isaac session into something
permanently true — including for a saved `EKF2_GPS_CTRL=0`, which is what boots
the next run GPS-denied on the pad. A log file is not worth a new safety
dependency.

## Consequences

With the replay topics recorded, `src/modules/replay` can re-run a flight through
EKF2 offline under different parameters, which breaks the one-hypothesis-per-flight
pattern of runs 1–5. Replay is **open-loop**: it re-estimates against recorded
sensor data and cannot change the trajectory that was flown, so it ranks
candidates and kills bad ones, and the live flight remains the only thing that can
pass 8.6.

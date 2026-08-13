# Task 8.6 passes on a non-growing excursion envelope over 180 s

The plan's original bar was "hover 60 s vision-only, holding station". Run 5
(2026-08-13) then produced a growing oscillation of roughly 40–60 s period, which
makes a 60 s test barely one period long: it can pass or fail on where in the
cycle the clock is stopped, and it cannot distinguish a damped mode from an
unstable one. 8.6 therefore passes when the aircraft holds vision-only for 180 s
(three or more periods) **and** the excursion envelope stops growing, with the
peak excursion recorded as a number rather than a verdict.

## Consequences

The bar is now stated in terms of *aircraft excursion* — where the airframe truly
is against where it was told to hold — not *estimator drift*, which is what the
telemetry has always shown. Those are different quantities and nothing in the
system computed the former; see ADR-0004.

# J10-A8.1 Y+/- Boundary Closure

This stage was read-only. No hardware motion or command frames were issued.

| candidate | predicted steps `[1..6]` | absolute margins `[1..6]` | min margin | limiter | IK | historical exceeded | maintenance required | decision |
|---|---|---|---:|:---:|:---:|:---:|:---:|---|
| Y+49.5 mm | `[1247,1989,1796,1796,1989,1247]` | `[753,11,204,204,11,753]` | 11 | ID2/ID5 | PASS | true | true | not executed |
| Y-45.75 mm | `[1981,1290,1437,1437,1290,1981]` | `[19,710,563,563,710,19]` | 19 | ID1/ID6 | PASS | true | true | not executed |

Both predicted targets are inside the authoritative `[0,2000]` range, but both have `absolute_margin_min < 20`. The `<20` criterion is only the endpoint-proximity search stop rule; it is not a qualification envelope. No endpoint command was sent.

## Closure

- Y+ maximum fully-qualified observed: **+49.0 mm** (ID2/ID5 margin 31).
- Y- maximum fully-qualified observed: **-45.5 mm** (ID1/ID6 margin 30).
- Positive predicted absolute-bound bracket: **+49.75 to +50.0 mm** (`+49.75` predicts endpoint 2000; `+50.0` predicts 2010).
- Negative predicted absolute-bound bracket: **-46.0 to -46.25 mm** (`-46.25` predicts 2002).
- Boundary classification: **pre-calibration absolute-endpoint-proximity censored**; no mechanical limit was observed.

Current observed XY workspace remains X `[-45.75,+45.75] mm`, Y `[-45.5,+49.0] mm`. No recommended safe limit is assigned pending all-DOF characterization.

No J7/J8/J9 production changes, controller tuning, or commit were made.

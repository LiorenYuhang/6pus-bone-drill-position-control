# J9-E1D-5R2 Static Gravity Reporting and Scientific Closure

Source raw artifact: `j9_e1d_5_raw.json`

## A. Acquisition/Data Status

- FIT dwells: 9/9 accepted
- VALIDATION dwells: 9/9 accepted
- Invalid dwell poses: `[]`
- H30/source/FK capture: clean in the original E1D-5 artifact

Accepted static dwells remain valid calibration data even though final PARK failed.

## B. Offline Static Gravity Result

Estimator: existing `solve_gravity_wahba()` using `u_P = R_P_IP * u_I`.

FIT:
- Euler XYZ deg: `[0.5243933631209545, 1.6836261199718294, 90.49480007831087]`
- Residual RMS rad: `0.0015437118445587878`
- Residual P95 rad: `0.0022547293289177464`
- Observability singular values: `[2.9972647023136236, 2.997259696139929, 0.1812142879576658]`
- Observability condition number: `16.53989172759836`

VALIDATION:
- Euler XYZ deg: `[0.5413774153789761, 1.6931399758811367, 91.5893795099681]`
- Residual RMS rad: `0.0016974531713613461`
- Residual P95 rad: `0.0025675947131653175`

FIT-vs-VALIDATION SO(3) delta deg: `1.0942520793147956`

Cross-run SO(3) deltas:
- FIT vs E1D-1 clean: `0.004591216228715042` deg
- FIT vs E1D-3: `0.09534362480029694` deg
- FIT vs E1C-2 dynamic: `2.6732224797019417` deg

## C. Hardware Closure Status

- Hardware teardown status: `FAIL`
- Final READ_ONLY status: `PASS`
- Final positions steps: `{'1': 59, '2': 61, '3': 59, '4': 60, '5': 60, '6': 60}`
- Final currents mA: `{'1': 0, '2': 0, '3': 0, '4': 0, '5': 0, '6': 0}`
- Final faults: `{'1': 0, '2': 0, '3': 0, '4': 0, '5': 0, '6': 0}`

Overall E1D-5 hardware closure remains FAIL because final PARK did not complete to VALID_PARK.

## D. Scientific Closure

- Static gravity FIT estimates are highly repeatable across E1D-1/E1D-3/E1D-5.
- Gravity-only data strongly supports roll/pitch consistency.
- Yaw-like orientation remains weakly observable.
- Static-vs-dynamic E1C discrepancy remains unresolved.
- Full `R_P_IP` is not frozen and was not written to runtime configuration.

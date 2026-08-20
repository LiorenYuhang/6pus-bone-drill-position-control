# UART and motor test log archive

## Permanent UART Phase baseline

- Status: `PASS`, permanent retention, never overwrite or delete.
- Absolute path: `/home/lyapunov/bone_drill/up/motor/uart_multi/logs/six_ids_test01.log`
- SHA-256: `a5c43ead04d26dfe80945cdd84dc3e3de9f686dc4b1bfef17b02e875620ffa15`
- Scope: six motors, IDs 1 through 6, initial validation plus 100 ordered polling rounds.

The baseline file stays at its original absolute path. The
`archive/uart_phase_baseline/README.md` marker records the acceptance result
without moving or duplicating the original log.

## Archive layout

```text
logs/
  six_ids_test01.log                 # immutable UART Phase baseline
  archive/uart_phase_baseline/       # baseline acceptance marker
  id1_motion/                        # independent ID1 motion experiments
```

Every later experiment must use a new traceable name such as
`YYYYMMDD_HHMMSS_phase_subject_runNN.log`. Never reuse an existing path. The
ID1 motion tool opens logs with exclusive creation and fails if the requested
file already exists.

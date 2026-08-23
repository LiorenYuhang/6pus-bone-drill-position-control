# J6 Unified Test Report Schema

All J6 tools should emit a JSON object using this schema. The schema is a
reporting contract; it does not authorize hardware access. Reports may contain
tool-specific `metrics`, `evidence`, and `observations`, but the common fields
must remain stable so the summary generator can consume reports without
modifying them.

## Common fields

| Field | Type | Required | Meaning |
| --- | --- | --- | --- |
| `schema_version` | integer | yes | Schema revision, currently `1` |
| `tool` | string | yes | Producing tool identifier |
| `phase` | string | yes | `J6_PHASE0` through `J6_PHASE8` |
| `timestamp` | ISO-8601 string | yes | Report creation time |
| `session_id` | string/null | recommended | Commissioning session correlation ID |
| `hardware_mode` | enum/null | yes | `mock`, `read_only`, `active`, or `offline` |
| `port` | string/null | recommended | Transport path, if applicable |
| `baudrate` | integer/null | recommended | Transport baudrate, if applicable |
| `hardware_motion_executed` | boolean | yes | True only for actual physical motion |
| `hardware_write_count` | integer | yes | Physical register writes; must be zero for read-only/offline tools |
| `pass` | boolean | yes | Tool-level verdict |
| `outcome` | enum/null | recommended | `PASS`, `FAIL`, `STOP`, `HOLD`, or `NOT_STARTED` |
| `pass_conditions` | object | yes | Named boolean gate results |
| `failure_reasons` | array[string] | yes | Human-readable failed/blocked conditions |
| `evidence` | object | recommended | Paths, hashes, measurements, or external record IDs |
| `observations` | object | recommended | Operator annotations not inferred by software |
| `metrics` | object | recommended | Tool-specific numeric summaries |
| `source_report_path` | string/null | recommended | Input report for wrappers/replays |
| `source_reports` | array[string] | recommended | Input reports for aggregators |

## Safety and provenance rules

1. A report must distinguish `hardware_write_count` from Mock bus activity. A
   Mock test may include a `mock_write_count`, but it must not claim physical
   writes or physical PASS.
2. A read-only report is valid only when `hardware_mode == "read_only"`, all
   lifecycle write counters are zero, and `hardware_motion_executed == false`.
3. A summary tool must preserve input paths and verdicts and must never rewrite
   source JSON, alter its timestamps, or merge away failed reports.
4. Physical measurements, direction, clearance, uncertainty, and signatures are
   evidence/observation fields, not silently inferred software metrics.
5. Unknown or missing required fields make a report `INVALID` in aggregation;
   they must not be treated as PASS.

## Canonical JSON skeleton

```json
{
  "schema_version": 1,
  "tool": "<producer>",
  "phase": "J6_PHASE<0-8>",
  "timestamp": "2026-08-22T00:00:00+00:00",
  "session_id": null,
  "hardware_mode": "mock",
  "port": null,
  "baudrate": null,
  "hardware_motion_executed": false,
  "hardware_write_count": 0,
  "pass": false,
  "outcome": "NOT_STARTED",
  "pass_conditions": {},
  "failure_reasons": [],
  "evidence": {},
  "observations": {},
  "metrics": {},
  "source_report_path": null,
  "source_reports": []
}
```

## Aggregated summary fields

The J6 summary generator emits a separate Markdown document with:

- summary creation time and input directory/glob;
- report count, phase coverage, PASS/FAIL/STOP/INVALID counts;
- per-phase verdict, tool, timestamp, source path, and failure reasons;
- explicit missing phases and invalid reports;
- physical-motion and write-count totals as reported by inputs;
- a provenance statement that source JSON files were read-only inputs.


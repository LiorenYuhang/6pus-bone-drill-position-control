#!/usr/bin/env python3
"""Generate a read-only Markdown summary from existing J6 JSON reports."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


PHASES = tuple(f"J6_PHASE{index}" for index in range(9))
REQUIRED_FIELDS = (
    "schema_version",
    "tool",
    "timestamp",
    "hardware_mode",
    "hardware_motion_executed",
    "hardware_write_count",
    "pass",
    "pass_conditions",
    "failure_reasons",
)
LEGACY_PHASE_BY_TOOL = {"j6_hardware_connection_checker": "J6_PHASE1"}
CODE = chr(96)


def _phase_number(phase: str) -> int:
    try:
        return int(phase.rsplit("PHASE", 1)[1])
    except (IndexError, ValueError):
        return 99


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("top-level JSON value must be an object")
    return value


def _normalize_report(raw: dict[str, Any], path: Path) -> dict[str, Any]:
    report = dict(raw)
    warnings: list[str] = []
    missing = [field for field in REQUIRED_FIELDS if field not in report]
    phase = report.get("phase")
    if phase is None and report.get("tool") in LEGACY_PHASE_BY_TOOL:
        phase = LEGACY_PHASE_BY_TOOL[report["tool"]]
        warnings.append(f"phase inferred from tool name: {phase}")
    if phase is not None:
        report["phase"] = phase
    if "phase" not in report:
        missing.append("phase")

    type_errors: list[str] = []
    if "schema_version" in report and not isinstance(report["schema_version"], int):
        type_errors.append("schema_version must be an integer")
    if "tool" in report and not isinstance(report["tool"], str):
        type_errors.append("tool must be a string")
    if "hardware_motion_executed" in report and not isinstance(report["hardware_motion_executed"], bool):
        type_errors.append("hardware_motion_executed must be boolean")
    if "hardware_write_count" in report and (
        not isinstance(report["hardware_write_count"], int)
        or isinstance(report["hardware_write_count"], bool)
        or report["hardware_write_count"] < 0
    ):
        type_errors.append("hardware_write_count must be a non-negative integer")
    if "pass" in report and not isinstance(report["pass"], bool):
        type_errors.append("pass must be boolean")
    if "pass_conditions" in report and not isinstance(report["pass_conditions"], dict):
        type_errors.append("pass_conditions must be an object")
    if "failure_reasons" in report and not isinstance(report["failure_reasons"], list):
        type_errors.append("failure_reasons must be an array")

    errors = [f"missing required field: {field}" for field in missing]
    errors.extend(type_errors)
    if "phase" in report and report.get("phase") not in PHASES:
        errors.append(f"invalid phase: {report.get('phase')!r}")
    if errors:
        status = "INVALID"
    elif report.get("pass") is True:
        status = "PASS"
    elif report.get("outcome") == "STOP":
        status = "STOP"
    else:
        status = "FAIL"

    report["_source_path"] = str(path)
    report["_status"] = status
    report["_validation_errors"] = errors
    report["_warnings"] = warnings
    return report


def collect_reports(
    *,
    input_dir: Path,
    pattern: str = "j6_*.json",
    input_paths: Iterable[Path] | None = None,
) -> list[dict[str, Any]]:
    """Read reports without mutating any source file."""
    paths = list(input_paths) if input_paths is not None else sorted(input_dir.glob(pattern))
    reports: list[dict[str, Any]] = []
    for path in sorted({Path(value) for value in paths}):
        try:
            reports.append(_normalize_report(_load_json(path), path))
        except Exception as exc:
            reports.append({
                "_source_path": str(path),
                "_status": "INVALID",
                "_validation_errors": [f"unable to read JSON: {exc}"],
                "_warnings": [],
                "tool": "<unreadable>",
                "phase": None,
                "timestamp": None,
                "hardware_motion_executed": None,
                "hardware_write_count": None,
                "failure_reasons": [],
            })
    return reports


def _md(value: Any) -> str:
    return str(value).replace("|", "\\|").replace("\n", " ")


def generate_markdown_summary(reports: list[dict[str, Any]], *, source_description: str) -> str:
    statuses = Counter(report.get("_status", "INVALID") for report in reports)
    phase_reports: dict[str, list[dict[str, Any]]] = {phase: [] for phase in PHASES}
    for report in reports:
        if report.get("phase") in phase_reports:
            phase_reports[report["phase"]].append(report)
    missing_phases = [phase for phase, items in phase_reports.items() if not items]
    physical_motion_count = sum(report.get("hardware_motion_executed") is True for report in reports)
    total_writes = sum(
        report["hardware_write_count"] for report in reports
        if isinstance(report.get("hardware_write_count"), int)
        and not isinstance(report.get("hardware_write_count"), bool)
    )

    lines = [
        "# J6 Commissioning Summary", "",
        f"Generated (UTC): {CODE}{datetime.now(timezone.utc).isoformat()}{CODE}",
        f"Source selection: {CODE}{_md(source_description)}{CODE}", "",
        "> This summary was generated from existing JSON reports. Source reports were read only and were not modified.",
        "", "## Overview", "",
        f"- Reports read: **{len(reports)}**",
        f"- PASS: **{statuses['PASS']}**",
        f"- FAIL: **{statuses['FAIL']}**",
        f"- STOP: **{statuses['STOP']}**",
        f"- INVALID: **{statuses['INVALID']}**",
        f"- Reports claiming physical motion: **{physical_motion_count}**",
        f"- Reported hardware writes (sum): **{total_writes}**", "",
        "## Phase Coverage", "",
        "| Phase | Reports | Latest status | Tool(s) |",
        "| --- | ---: | --- | --- |",
    ]
    for phase in PHASES:
        items = sorted(phase_reports[phase], key=lambda report: str(report.get("timestamp") or ""))
        latest = items[-1] if items else None
        tools = ", ".join(sorted({_md(item.get("tool", "")) for item in items}))
        lines.append(f"| {phase} | {len(items)} | {_md(latest.get('_status') if latest else 'MISSING')} | {tools} |")
    lines.extend(["", "Missing phases: " + (", ".join(f"{CODE}{phase}{CODE}" for phase in missing_phases) if missing_phases else "none"), "", "## Report Details", "", "| Status | Phase | Tool | Timestamp | Source |", "| --- | --- | --- | --- | --- |"])
    for report in sorted(reports, key=lambda item: (_phase_number(str(item.get("phase"))), str(item.get("timestamp") or ""), str(item.get("_source_path")))):
        lines.append(f"| {_md(report.get('_status'))} | {_md(report.get('phase', 'MISSING'))} | {_md(report.get('tool', '<unknown>'))} | {_md(report.get('timestamp'))} | {CODE}{_md(report.get('_source_path'))}{CODE} |")
        errors = list(report.get("_validation_errors", [])) + [str(reason) for reason in report.get("failure_reasons", [])]
        warnings = list(report.get("_warnings", []))
        for error in errors:
            lines.append(f"- {CODE}{_md(report.get('tool', '<unknown>'))}{CODE} issue: {_md(error)}")
        for warning in warnings:
            lines.append(f"- {CODE}{_md(report.get('tool', '<unknown>'))}{CODE} warning: {_md(warning)}")
    lines.extend([
        "", "## Interpretation", "",
        "- Phase gates remain strictly sequential; report presence does not imply gate PASS.",
        "- INVALID reports and missing phases require review before claiming commissioning progress.",
        "- Mock, synthetic, replay, and same-board reports are not physical-mechanism evidence.",
        "- Physical measurements, operator observations, uncertainty, and signatures remain external evidence.", "",
    ])
    return "\n".join(lines)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate a read-only J6 Markdown summary")
    parser.add_argument("--input-dir", type=Path, default=Path(__file__).resolve().parents[2] / "log")
    parser.add_argument("--glob", default="j6_*.json")
    parser.add_argument("--input", type=Path, action="append", dest="input_paths")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[2] / "log" / "j6_commissioning_summary.md")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    reports = collect_reports(input_dir=args.input_dir, pattern=args.glob, input_paths=args.input_paths)
    source_description = ", ".join(str(path) for path in args.input_paths) if args.input_paths else f"{args.input_dir}/{args.glob}"
    summary = generate_markdown_summary(reports, source_description=source_description)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(summary, encoding="utf-8")
    print(f"Wrote J6 summary: {args.output} ({len(reports)} reports)")
    return 0 if reports and all(report["_status"] == "PASS" for report in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())

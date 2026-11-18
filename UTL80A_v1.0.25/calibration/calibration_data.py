"""Read acquisition workbooks without changing their cells."""

from dataclasses import dataclass
from datetime import datetime, time
from itertools import groupby
import os
from pathlib import Path
import re

import numpy as np
from openpyxl import load_workbook

CHANNELS = tuple(f"L{i}" for i in range(1, 9))
TEMPS = tuple(f"TMP117_{i}" for i in range(1, 5)) + ("TMP275",)
BASE = ("Time", "Target_Axis", "Applied_Load") + CHANNELS + TEMPS
STEP = ("Elapsed_Time", "Repeat", "Load_Unit", "State")
INVALID_RAW = (1 << 28) - 1


@dataclass
class Run:
    source: str
    sheet: str
    kind: str
    axis: str
    load: float
    unit: str
    rows: list

    @property
    def key(self):
        return f"{self.source}::{self.sheet}"


def number(value):
    try:
        return float(value) if value is not None else np.nan
    except (TypeError, ValueError):
        return np.nan


def clock(value):
    if isinstance(value, datetime):
        value = value.time()
    if isinstance(value, time):
        return value.hour * 3600 + value.minute * 60 + value.second + value.microsecond / 1e6
    try:
        h, m, s = map(float, str(value).strip().split(":"))
        if 0 <= h < 24 and 0 <= m < 60 and 0 <= s < 60:
            return h * 3600 + m * 60 + s
    except (ValueError, TypeError):
        pass
    return np.nan


def import_runs(root):
    runs, issues = [], []
    files = []
    for directory, folders, names in os.walk(root):
        folders[:] = [folder for folder in folders if folder not in {"Tool", "node_modules", ".git", ".codex"}]
        files.extend(Path(directory) / name for name in names
                     if name.lower().endswith(".xlsx") and not name.startswith("~$"))
    files.sort()
    for path in files:
        source = path.relative_to(root).as_posix()
        book = load_workbook(path, read_only=True, data_only=True)
        try:
            for sheet in book:
                records = enumerate(sheet.values, 1)
                header = next(records, (1, ()))[1]
                if not header or not any(v is not None for v in header):
                    issues.append(dict(Source_File=source, Sheet_Name=sheet.title, Issue="empty_sheet"))
                    continue
                fields = {v: i for i, v in enumerate(header) if v is not None}
                step = any(f in fields for f in STEP)
                required = BASE + STEP if step else BASE
                missing = set(required) - fields.keys()
                if missing or len(fields) != sum(v is not None for v in header):
                    issues.append(dict(Source_File=source, Sheet_Name=sheet.title,
                                       Issue="invalid_header " + ",".join(sorted(missing))))
                    continue
                rows = [dict(zip(header, cells), source_row=i) for i, cells in records
                        if any(v is not None for v in cells)]
                axes = {str(r["Target_Axis"]).strip() for r in rows}
                if len(axes) != 1:
                    issues.append(dict(Source_File=source, Sheet_Name=sheet.title, Issue="missing_or_mixed_axes"))
                    continue
                axis = axes.pop()
                if step:
                    units = {str(r["Load_Unit"]).strip().lower() for r in rows}
                    states = {r["State"] for r in rows}
                    torque = axis in {"Tz+", "Tz-"}
                    expected = {"Unloaded", "Torque" if torque else "Loaded"}
                    if units != ({"nm"} if torque else {"kg"}) or states != expected:
                        issues.append(dict(Source_File=source, Sheet_Name=sheet.title, Issue="invalid_states_or_units"))
                        continue
                    kind = "tz_step" if torque else ("fxy_step" if axis[:2] in {"Fx", "Fy"} else "txy_fz_step")
                    loads = {number(r["Applied_Load"]) for r in rows if r["State"] != "Unloaded"}
                    unit = "Nm" if torque else "kg"
                else:
                    kind = "fxy_hold" if axis.startswith(("Fx", "Fy")) else "txy_fz_hold"
                    loads = {number(r["Applied_Load"]) for r in rows}
                    unit = "kg (experiment log)"
                if len(loads) != 1 or not all(np.isfinite(v) and v >= 0 for v in loads):
                    issues.append(dict(Source_File=source, Sheet_Name=sheet.title, Issue="missing_or_mixed_loads"))
                    continue
                runs.append(Run(source, sheet.title, kind, axis, loads.pop(), unit, rows))
        finally:
            book.close()
    return runs, issues


def row_arrays(rows, step):
    raw = np.array([[number(r[n]) for n in CHANNELS] for r in rows])
    temp = np.array([[number(r[n]) for n in TEMPS] for r in rows])
    times = np.array([number(r["Elapsed_Time"]) if step else clock(r["Time"]) for r in rows])
    if not step:
        midnight = np.r_[0, np.cumsum(np.diff(times) < -43200)]
        times += midnight * 86400
    finite = np.flatnonzero(np.isfinite(times))
    if finite.size:
        times -= times[finite[0]]
    return times, raw, temp


def quality(rows, step):
    times, raw, temp = row_arrays(rows, step)
    issues = [[] for _ in rows]
    keep = np.ones(len(rows), dtype=bool)

    def mark(mask, reason, exclude=False):
        for i in np.flatnonzero(mask):
            issues[i].append((reason, exclude))
        if exclude:
            keep[mask] = False

    for j, channel in enumerate(CHANNELS):
        x = raw[:, j]
        mark(~np.isfinite(x), f"{channel}:missing_or_nonnumeric", True)
        mark(x == 0, f"{channel}:zero", True)
        mark(((x < 0) | (x >= INVALID_RAW) | (np.isfinite(x) & (x != np.trunc(x)))), f"{channel}:invalid_raw28", True)
    for j, channel in enumerate(TEMPS):
        mark(~np.isfinite(temp[:, j]), f"{channel}:missing_or_nonnumeric", True)
        mark(temp[:, j] == 0, f"{channel}:zero", True)
    mark(~np.isfinite(times) | (times < 0), "invalid_timestamp", True)
    dt = np.diff(times, prepend=np.nan)
    positive = dt[np.isfinite(dt) & (dt > 0)]
    median_dt = np.median(positive) if positive.size else np.nan
    mark(dt < 0, "timestamp_reversal", True)
    duplicate = np.r_[False, np.all(raw[1:] == raw[:-1], axis=1) & np.all(temp[1:] == temp[:-1], axis=1)]
    mark((dt == 0) & duplicate, "duplicate_timestamp_and_values", True)
    mark((dt == 0) & ~duplicate, "timestamp_collision")
    mark(dt > 5 * median_dt, "timestamp_gap_gt_5_median_periods")

    # Flag abrupt changes without deleting valid motion, especially before a wrench click.
    eligible = np.flatnonzero(keep)
    if len(eligible) > 2:
        valid_raw = raw[eligible]
        differences = np.diff(valid_raw, axis=0)
        center = np.median(differences, axis=0)
        spread = 1.4826 * np.median(np.abs(differences - center), axis=0)
        # A nonzero baseline avoids interpreting normal ADC quantization as an outlier.
        spread = np.maximum(spread, np.quantile(np.abs(differences), 0.9, axis=0))
        jump = (np.abs(differences - center) > 12 * spread) & (spread > 0)
        for k, j in zip(*np.where(jump)):
            issues[eligible[k + 1]].append((f"{CHANNELS[j]}:abrupt_change", False))
        for j, channel in enumerate(CHANNELS):
            x = valid_raw[:, j]
            blocks = np.split(np.arange(len(x)), np.flatnonzero(np.diff(x) != 0) + 1)
            for block in blocks:
                first, last = eligible[block[[0, -1]]]
                if times[last] - times[first] >= 2:
                    issues[first].append((f"{channel}:flat_until_excel_row_{rows[last]['source_row']}", False))
    return times, raw, temp, keep, issues


def pairs(run, required_state):
    """Yield contiguous acquisitions, retaining distinct appended batches."""
    batch, previous, pending = 0, 0, None
    keys = lambda r: (r["Repeat"], r["State"])
    for (repeat, state), group in groupby(run.rows, keys):
        segment = list(group)
        numeric_repeat = number(repeat)
        if not np.isfinite(numeric_repeat) or not numeric_repeat.is_integer() or numeric_repeat < 1:
            yield None, "invalid_repeat", segment, segment[:0]
            continue
        repeat = int(numeric_repeat)
        if state == "Unloaded":
            if pending is not None:
                yield pending[0], "missing_loaded_segment", pending[1], []
            if repeat <= previous:
                batch += 1
            previous = repeat
            pending = ((batch + 1, repeat), segment)
        elif state == required_state and pending and pending[0][1] == repeat:
            unloaded = pending[1]
            if any(number(r["Applied_Load"]) != 0 for r in unloaded):
                yield pending[0], "nonzero_unloaded_applied_load", unloaded, segment
            else:
                yield pending[0], "", unloaded, segment
            pending = None
        else:
            yield (batch + 1, repeat), "unpaired_loaded_segment", [], segment
    if pending:
        yield pending[0], "missing_loaded_segment", pending[1], []


def safe_name(name):
    return re.sub(r"[^a-zA-Z0-9_.+-]", "_", name)

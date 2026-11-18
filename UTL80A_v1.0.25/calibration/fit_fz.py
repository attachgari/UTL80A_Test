"""Fit a signed eight-coil Fz row with optional coupled simulation cases."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from calibration_analysis import inspect_window
from calibration_data import import_runs, pairs, quality

REPO = Path(__file__).resolve().parents[1]
SKETCH = REPO / "arduino" / "UTL80A_v1.0.25"
G = 9.80665
RAW_SCALE = 5000.
RAW_FIELDS = tuple(f"Delta_L{i}" for i in range(1, 9))
WRENCH_FIELDS = ("Fx_N", "Fy_N", "Fz_N", "Tx_Nm", "Ty_Nm", "Tz_Nm")


def direct_steps(data):
    runs, _ = import_runs(data / "261116_TxyFz_5S")
    for run in runs:
        if run.kind != "txy_fz_step" or run.axis not in ("Fz", "Fz+", "Fz-"):
            continue
        for pair, issue, unloaded, loaded in pairs(run, "Loaded"):
            if issue:
                continue
            batch, repeat = pair
            t0, raw0, _, ok0, _ = quality(unloaded, True)
            indices, _ = inspect_window(t0, raw0, ok0)
            t, raw, _, ok, _ = quality(loaded, True)
            if len(indices) < 12 or ok.sum() < 50:
                raise ValueError(f"{run.key}, batch {batch}, repeat {repeat}: too few valid samples")
            tail = ok & (t >= t[ok][-1] - 1)
            delta = raw[tail].mean(axis=0) - raw0[indices].mean(axis=0)
            force = run.load * G * (1 if run.axis == "Fz+" else -1)
            yield (run.key, batch, repeat, force, delta)


def known_loads(reference_csv):
    with reference_csv.open(encoding="utf-8-sig") as file:
        rows = [r for r in csv.DictReader(file) if r["Calibration_Eligible"] == "True"]
    x = np.array([[float(row[name]) for name in RAW_FIELDS] for row in rows])
    labels = np.array([float(row["Fz"]) for row in rows])
    click = np.array([row["Load_Unit"] == "Nm" for row in rows])
    return x, labels, click


def open_simulation(path):
    if path is None:
        return np.empty((0, 8)), np.empty((0,))
    with path.open(encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)
        if not set((*WRENCH_FIELDS, *RAW_FIELDS)).issubset(reader.fieldnames or ()):
            raise ValueError("Simulation CSV requires the six wrench fields and Delta_L1 through Delta_L8")
        rows = [[float(row[name]) for name in (*WRENCH_FIELDS, *RAW_FIELDS)] for row in reader]
    values = np.array(rows)
    if values.size == 0 or not np.isfinite(values).all():
        raise ValueError("Supply at least one case with finite loads and all eight simulated deltas")
    return values[:, 6:], values[:, 2]


def fz_zero(data):
    runs, _ = import_runs(data / "261116_TxyFz_300S")
    run = next(run for run in runs if run.sheet == "Fz_0")
    t, raw, _, valid, _ = quality(run.rows, False)
    noise = raw[valid & (t >= 180) & (t < 240)]
    tare = raw[valid & (t >= 240) & (t < 250)]
    test = raw[valid & (t >= 250) & (t < 300)]
    if min(len(noise), len(tare), len(test)) < 50:
        raise ValueError("Fz_0 is missing the noise, tare, or validation interval")
    return noise - noise.mean(axis=0), test - tare.mean(axis=0)


def fit(x, fz, zero, refs, labels, click, simulation, sim_fz):
    """Solve with scaled counts and force labels in N, then return N/count."""
    x, zero, refs, simulation = (value / RAW_SCALE for value in (x, zero, refs, simulation))
    w = np.where(click, .20, 1.)
    gram = x.T @ x / len(x) + .003 * np.eye(8) + 20 * zero.T @ zero / len(zero)
    rhs = x.T @ fz / len(x) + .10 * refs.T @ (w * labels) / w.sum()
    gram += .10 * refs.T @ (w[:, None] * refs) / w.sum()
    if len(simulation):
        # The entire simulation set carries 0.05 times the total direct-measurement weight.
        gram += .05 * simulation.T @ simulation / len(simulation)
        rhs += .05 * simulation.T @ sim_fz / len(simulation)
    return np.linalg.solve(gram, rhs) / RAW_SCALE


def rms(error):
    return float(np.sqrt(np.mean(error**2)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", required=True, type=Path, help="Path to the TestData_1116 measurement folder")
    parser.add_argument("--reference-csv", required=True, type=Path, help="Path to the existing calibration experiments.csv")
    parser.add_argument("--simulation", type=Path, help="Optional CSV of six-axis loads and all eight ANSYS-derived RAW deltas")
    args = parser.parse_args()
    cases = list(direct_steps(args.data_dir))
    if not cases:
        raise ValueError("No usable direct Fz steps")
    x = np.array([row[4] for row in cases])
    fz = np.array([row[3] for row in cases])
    zero, zero_test = fz_zero(args.data_dir)
    refs, labels, click = known_loads(args.reference_csv)
    sim_r, sim_fz = open_simulation(args.simulation)
    constraints = (zero, refs, labels, click, sim_r, sim_fz)
    coefficient = fit(x, fz, *constraints)
    result = {}
    for name, split in (
        ("hold_repeat", [row[2] for row in cases]),
        ("hold_batch", [f"{row[0]}::{row[1]}" for row in cases]),
        ("hold_level", fz),
    ):
        split = np.array(split)
        prediction = np.zeros_like(fz)
        for group in np.unique(split):
            selected = split == group
            training = ~selected
            prediction[selected] = x[selected] @ fit(x[training], fz[training], *constraints)
        result[name + "_rmse_N"] = rms(prediction - fz)
        if name == "hold_batch":
            result["hold_batch_by_source"] = [
                {"source": str(group), "reference_N": float(fz[split == group].mean()),
                 "predicted_mean_N": float(prediction[split == group].mean()),
                 "repeat_count": int((split == group).sum())}
                for group in np.unique(split)
            ]
    result.update(direct_repeats=len(cases), simulation_cases=len(sim_r),
                  fitted_direct_rmse_N=rms(x @ coefficient - fz),
                  unloaded_250_300_std_N=float((zero_test @ coefficient).std()),
                  unloaded_250_300_mean_N=float((zero_test @ coefficient).mean()),
                  nominal_zero_fz_during_tz_peak_rms_N=rms(refs[click] @ coefficient),
                  coefficients_N_per_raw_count=coefficient.tolist(),
                  qualification="Direct vertical deadweight comparison assumes the logged mass acts along negative Fz. Other direct-step wrench components were not assigned labels.")
    coefficients = ", ".join(f"{v:.10e}f" for v in coefficient)
    (SKETCH / "fz_calibration.h").write_text(
        "#pragma once\n\n// Generated by fit_fz.py. N per RAW count, L1 to L8.\n"
        f"static constexpr float FZ_DIRECT_COEFFICIENTS[8] = {{{coefficients}}};\n",
        encoding="utf-8")
    (REPO / "reports" / "fz_validation_v1.0.25.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()

"""Train and export the UTL80A causal residual TCN from the project calibration data."""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

from calibration_analysis import inspect_window
from calibration_data import import_runs, pairs, quality


REPO = Path(__file__).resolve().parents[1]
SKETCH = REPO / "arduino" / "UTL80A_v1.0.25"

AXES = ("Fx", "Fy", "Fz", "Ty", "Tx", "Tz")  # Worksheet order -> firmware output
OUTPUT_SCALE = np.array([9.80665, 9.80665, 9.80665, .392266, .392266, 1.])
LAGS = .020 * np.arange(7)
RIDGE = .02
L2 = .003
HIDDEN = 8
EPOCHS = 320


def window(t, raw, endpoint):
    query = t[endpoint] - LAGS
    t, last = np.unique(t[:endpoint + 1][::-1], return_index=True)
    raw = raw[endpoint - last]
    if query[-1] < t[0]:
        return None
    right = np.minimum(np.searchsorted(t, query), len(t) - 1)
    left = np.maximum(right - 1, 0)
    if np.any((t[right] - t[left] > .065) & (t[right] != query)):
        return None
    return np.column_stack([np.interp(query, t, channel) for channel in raw.T])


def dataset(data, summary):
    known = defaultdict(list)
    with summary.open(encoding="utf-8-sig", newline="") as file:
        for ref in csv.DictReader(file):
            if ref["Calibration_Eligible"] == "True":
                known[ref["Test_ID"].split(":peak")[0]].append(ref)
    samples = []
    for folder in ("261116_Fxy_5S", "261116_TxyFz_5S", "261116_Tz"):
        runs, _ = import_runs(data / folder)
        for run in runs:
            torque = run.kind == "tz_step"
            if not torque and run.kind not in ("fxy_step", "txy_fz_step"):
                continue
            for pair, issue, unloaded, loaded in pairs(run, "Torque" if torque else "Loaded"):
                if issue:
                    continue
                batch, repeat = pair
                key = f"TestData_1116/{folder}/{run.key}::{batch}:{repeat}"
                if key not in known:
                    continue
                metadata = (key, repeat, run.load, run.axis, torque)
                t0, raw0, _, ok0, _ = quality(unloaded, True)
                t, raw, _, ok, _ = quality(loaded, True)
                ref, _ = inspect_window(t0, raw0, ok0)
                zero = raw0[ref].mean(axis=0)
                # These workbooks timestamp host serial reads in bursts. The capture
                # firmware emits every 20 ms, so preserve the original row spacing.
                t0, raw0 = np.flatnonzero(ok0) * .020, raw0[ok0] - zero
                t, raw = np.flatnonzero(ok) * .020, raw[ok] - zero
                for i in np.unique(np.linspace(len(t0) // 2, len(t0) - 1, 6).astype(int)):
                    history = window(t0, raw0, i)
                    if history is not None:
                        samples.append((*metadata, "zero", history, np.zeros(6)))
                for ref in known[key]:
                    y = np.array([float(ref[a]) for a in AXES])
                    if torque:
                        excel = np.array([r["source_row"] for r in loaded])[ok]
                        indices = np.flatnonzero(excel == int(ref["Excel_Row"]))
                        assert len(indices) == 1, ref["Test_ID"]
                    else:
                        tail = np.flatnonzero(t >= t[-1] - 1)
                        indices = np.unique(np.linspace(tail[0], tail[-1], 12).astype(int))
                    before = len(samples)
                    for i in indices:
                        history = window(t, raw, i)
                        if history is not None:
                            samples.append((*metadata, "click" if torque else "load", history, y))
                    if len(samples) == before:
                        raise ValueError(f"No valid causal window for {ref['Test_ID']}")
    return samples


def design(x, temporal):
    if not temporal:
        x = np.broadcast_to(x[:, :1], x.shape)
    taps = np.arange(3)[:, None] * 2 + np.arange(3)
    return x[:, taps].reshape(len(x), 3, 24)


def network(d, weights):
    first, second, head = weights
    h1 = np.tanh(d @ first).reshape(len(d), -1)
    h2 = np.tanh(h1 @ second)
    return h1, h2, h2 @ head


def train(d, residual, weight):
    rng = np.random.default_rng(5521)
    params = [rng.normal(0, .5 / np.sqrt(m), (m, n))
              for m, n in ((24, HIDDEN), (3 * HIDDEN, HIDDEN), (HIDDEN, 6))]
    moments = [np.zeros_like(p) for p in params]
    squares = [np.zeros_like(p) for p in params]
    d_flat = d.reshape(-1, 24)
    for step in range(1, EPOCHS + 1):
        h1, h2, pred = network(d, params)
        err = pred - residual
        # Huber, delta = 0.35 of each axis normalization scale.
        grad = weight * np.clip(err, -.35, .35)
        dh2 = (grad @ params[2].T) * (1 - h2**2)
        dh1 = (dh2 @ params[1].T) * (1 - h1**2)
        gradients = [d_flat.T @ dh1.reshape(-1, HIDDEN), h1.T @ dh2, h2.T @ grad]
        for param, gradient, moment, square in zip(params, gradients, moments, squares):
            gradient += L2 * param
            moment *= .9
            moment += .1 * gradient
            square *= .999
            square += .001 * gradient**2
            param -= .006 * (moment / (1 - .9**step)) / (np.sqrt(square / (1 - .999**step)) + 1e-8)
    return params


def ridge(x, y, w, strength):
    return np.column_stack([
        np.linalg.solve(x.T @ (w[:, a:a+1] * x) + strength * np.eye(x.shape[1]),
                        x.T @ (w[:, a] * y[:, a]))
        for a in range(y.shape[1])
    ])


def weights(keys, state):
    group_counts = defaultdict(int)
    for key, label in zip(keys, state):
        group_counts[key, label] += 1
    w = np.array([(.30 if label == "zero" else 1.) / group_counts[key, label]
                  for key, label in zip(keys, state)])
    w /= w.sum()
    output = np.repeat(w[:, None], 6, axis=1)
    # Click tool nominally specifies only Tz. Other axes are weak zero labels.
    output[state == "click", :5] *= .25
    return output


def fit(x, y, train_mask, w):
    x_fit, y_fit = x[train_mask], y[train_mask]
    scale = np.maximum(256., np.sqrt(np.mean(x_fit[:, 0]**2, axis=0)))
    xn = x / scale
    linear = ridge(xn[train_mask, 0], y_fit, w, RIDGE)
    fir = ridge(xn[train_mask].reshape(len(x_fit), -1), y_fit, w, RIDGE)
    baseline = xn[:, 0] @ linear
    residual = y_fit - baseline[train_mask]
    estimates = {"linear": baseline, "linear_fir": xn.reshape(len(x), -1) @ fir}
    for temporal, name in ((False, "snapshot_nonlinear"), (True, "tcn")):
        features = design(xn, temporal)
        params = train(features[train_mask], residual, w)
        estimates[name] = baseline + network(features, params)[2]
    return estimates, (scale, linear, *params)


def header(path, model):
    scale, linear, first, second, head = model
    tensors = dict(TCN_INV_RAW_SCALE=1 / scale, TCN_LINEAR=linear.T,
                   TCN_CONV1=first.T, TCN_CONV2=second.T,
                   TCN_HEAD=(head * OUTPUT_SCALE).T, TCN_OUTPUT_SCALE=OUTPUT_SCALE)

    def values(array):
        if array.ndim == 1:
            return "{" + ", ".join(f"{v:.9e}f" for v in array) + "}"
        return "{\n    " + ",\n    ".join(map(values, array)) + "\n}"

    lines = ["#pragma once", "", "// Generated by train_tcn.py. Fx, Fy, Fz, Tx, Ty, Tz."]
    for name, tensor in tensors.items():
        sizes = "".join(f"[{n}]" for n in tensor.shape)
        lines.append(f"static constexpr float {name}{sizes} = {values(tensor)};")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def evaluate(y, pred, state, keys, axis):
    err = (pred - y) * OUTPUT_SCALE
    static = state == "load"
    click = state == "click"
    zero = state == "zero"
    means = np.array([err[static & (keys == key)].mean(0) for key in sorted(set(keys[static]))])
    rms = lambda v: np.sqrt(np.mean(np.square(v), axis=0)).round(4).tolist()
    return dict(static_rmse=rms(means), unloaded_rmse=rms(err[zero]),
                pure_fx_into_fy=rms(err[static & np.char.startswith(axis, "Fx"), 1]),
                pure_fy_into_fx=rms(err[static & np.char.startswith(axis, "Fy"), 0]),
                click_rmse=rms(err[click]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", required=True, type=Path, help="Path to the TestData_1116 measurement folder")
    parser.add_argument("--reference-csv", required=True, type=Path, help="Path to the existing calibration experiments.csv")
    args = parser.parse_args()
    samples = dataset(args.data_dir, args.reference_csv)
    key, repeat, load, axis, torque, state, x, y = map(np.array, zip(*samples))
    y /= OUTPUT_SCALE
    results = {}
    for label, split in (("repeat", repeat), ("load", np.where(torque, -load, load))):
        predictions = {}
        for group in np.unique(split):
            test = split == group
            train_mask = ~test
            estimates, _ = fit(x, y, train_mask, weights(key[train_mask], state[train_mask]))
            for name, values in estimates.items():
                predictions.setdefault(name, np.zeros_like(y))[test] = values[test]
            print(label, group, int(test.sum()), flush=True)
        results[label] = {name: evaluate(y, values, state, key, axis)
                          for name, values in predictions.items()}
    estimates, model = fit(x, y, np.ones(len(x), dtype=bool), weights(key, state))
    results["counts"] = dict(static_runs=len(set(key[state == "load"])),
                             click_peaks=int((state == "click").sum()),
                             static_windows=int((state == "load").sum()),
                             unloaded_windows=int((state == "zero").sum()))
    assert results["counts"]["static_runs"] == 100, results["counts"]
    assert results["counts"]["click_peaks"] == 50, results["counts"]
    x_probe = np.stack([x[0], x[len(x)//2], x[-1]])
    probe_prediction = estimates["tcn"][[0, len(x)//2, len(x)-1]] * OUTPUT_SCALE
    results["firmware_replay"] = {"raw_windows_current_first": x_probe.tolist(),
                                  "expected": probe_prediction.tolist()}
    header(SKETCH / "tcn_weights.h", model)
    (REPO / "reports" / "tcn_validation_v1.0.24.json").write_text(
        json.dumps(results, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()

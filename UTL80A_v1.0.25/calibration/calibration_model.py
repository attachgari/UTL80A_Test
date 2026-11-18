"""Mechanical reference inputs, identifiable least-squares models and validation."""

import json
from pathlib import Path

import numpy as np

G = 9.80665
R = 0.04
AXES = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
UNITS = ("N", "N", "N", "N m", "N m", "N m")
DESIGN_FS = np.array([500, 500, 900, 20, 20, 20])
POSITIONS = {"Tx+": "-Y", "Tx-": "+Y", "Ty+": "+X", "Ty-": "-X"}
POINTS = {"+X": (R, 0, .0054), "-X": (-R, 0, .0054),
          "+Y": (0, R, .0054), "-Y": (0, -R, .0054)}


def wrench_txy(position, mass_kg):
    force = np.array([0, 0, -mass_kg * G])
    return np.r_[force, np.cross(POINTS[position], force)]


def read_geometry(path):
    if path is None:
        return {}
    geometry = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(geometry, dict):
        raise ValueError("Geometry must map source::sheet to experiment metadata")
    return geometry


def vec(values, n):
    array = np.asarray(values, dtype=float)
    if array.shape != (n,) or not np.isfinite(array).all():
        raise ValueError(f"Expected {n} finite numbers")
    return array


def reference(run, geometry):
    """Return incremental wrench and its provenance. Unconfirmed references are blank."""
    if run.axis in POSITIONS and run.kind == "txy_fz_step":
        position = POSITIONS[run.axis]
        return wrench_txy(position, run.load), position, (0, 0, -1), POINTS[position], "40 mm vertical standard mass"
    config = geometry.get(run.key)
    if not config or run.kind == "tz_step":
        reason = ("click setting without synchronized reference torque" if run.kind == "tz_step"
                  else "confirm mass and centered force line" if run.axis == "Fz"
                  else "confirm DUT force vector and line of action")
        return None, "", None, None, reason
    if not config.get("evidence"):
        raise ValueError(f"{run.key}: confirmed reference needs an evidence description")
    if "wrench" in config:
        return vec(config["wrench"], 6), config.get("position", ""), None, None, config["evidence"]
    point = vec(config["point_dut_m"], 3)
    if "world_to_dut" in config:
        rotation = np.asarray(config["world_to_dut"], dtype=float)
        if (rotation.shape != (3, 3) or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)):
            raise ValueError(f"{run.key}: world_to_dut must be a proper orthogonal rotation")
        direction = rotation @ np.array([0, 0, -1])
    else:
        direction = vec(config["force_direction_dut"], 3)
    if not np.isclose(np.linalg.norm(direction), 1, atol=1e-6):
        raise ValueError(f"{run.key}: force direction must be a unit vector")
    force = run.load * G * direction
    return np.r_[force, np.cross(point, force)], config.get("position", ""), direction, point, config["evidence"]


def svd_info(values):
    s = np.linalg.svd(values, compute_uv=False)
    return dict(rank=int(np.linalg.matrix_rank(values)), singular_values=s.tolist(),
                condition=float(s[0] / s[-1]) if s.size and s[-1] else None)


def fit(x, y, method, noise=None):
    # A common scaling preserves the units and relative weight of the eight coils.
    scale = 10000.0
    z = x / scale
    w = np.ones(len(x))
    if method == "WLS":
        # Delta mean uncertainty proxy, bounded so unusually quiet records cannot dominate.
        w = np.median(noise) / np.maximum(noise, np.finfo(float).eps)
        w = np.clip(w, .25, 4)
    a = np.c_[z, np.ones(len(z))] if method == "intercept" else z
    target = y
    if method.startswith("ridge_"):
        strength = float(method.split("_")[1])
        a = np.vstack([a, np.sqrt(strength) * np.eye(z.shape[1])])
        target = np.vstack([y, np.zeros((z.shape[1], y.shape[1]))])
        w = np.r_[w, np.ones(z.shape[1])]
    solved = np.linalg.lstsq(a * np.sqrt(w[:, None]), target * np.sqrt(w[:, None]), rcond=None)[0]
    intercept = solved[-1] if method == "intercept" else np.zeros(y.shape[1])
    return solved[:z.shape[1]].T / scale, intercept


def fit_and_validate(experiments):
    known = [r for r in experiments if r["Calibration_Eligible"]]
    if not known:
        return None
    x = np.array([[r[f"Delta_L{i}"] for i in range(1, 9)] for r in known])
    y_full = np.array([[r[a] for a in AXES] for r in known])
    active = np.flatnonzero(np.max(np.abs(y_full), axis=0) > 1e-12)
    y = y_full[:, active]
    refs = np.max(np.abs(y), axis=0)
    noise = np.array([r["Delta_Mean_Variance"] for r in known])
    physical_rank = np.linalg.matrix_rank(y / refs)
    if physical_rank != len(active) or np.linalg.matrix_rank(x) < physical_rank:
        raise ValueError("The references or RAW28 features cannot resolve the active axes")
    names = [AXES[j] for j in active]
    methods = ("OLS", "intercept", "WLS", "ridge_0.001", "ridge_0.01", "ridge_0.1", "ridge_1")
    splits = {"leave_repeat_out": [r["Repeat"] for r in known],
              "leave_load_out": [f"{r['Load_Unit']}:{r['Applied_Load']}" for r in known],
              "leave_workbook_out": [r["Source_File"] for r in known]}
    evaluations, predictions = [], []
    for method in methods:
        for scheme, groups in splits.items():
            p = np.full_like(y, np.nan)
            for group in sorted(set(groups)):
                held = np.array([g == group for g in groups])
                if not (~held).any() or np.linalg.matrix_rank(y[~held] / refs) < physical_rank:
                    continue
                c, intercept = fit(x[~held], y[~held], method, noise[~held])
                p[held] = x[held] @ c.T + intercept
            mask = np.isfinite(p).all(axis=1)
            if not mask.any():
                continue
            error = p[mask] - y[mask]
            score = np.sqrt(np.mean((error / refs) ** 2))
            for j, axis in enumerate(names):
                e = error[:, j]
                evaluations.append(dict(Model=method, Validation=scheme, Axis=axis, Samples=len(e),
                                        RMSE=float(np.sqrt(np.mean(e**2))), MAE=float(np.mean(np.abs(e))),
                                        Max_Abs=float(np.max(np.abs(e))), Bias=float(np.mean(e)),
                                        Max_Percent_Design_FS=float(100 * np.max(np.abs(e)) / DESIGN_FS[active[j]]),
                                        Normalized_Joint_RMSE=float(score)))
            for index in np.flatnonzero(mask):
                for j, axis in enumerate(names):
                    r = known[index]
                    predictions.append(dict(Model=method, Validation=scheme, Held_Out=groups[index],
                                            Test_ID=r["Test_ID"], Axis=axis, Target_Axis=r["Target_Axis"],
                                            Mass_or_Setting=r["Applied_Load"], Load_Unit=r["Load_Unit"], TMP275=r["TMP275"],
                                            Reference=y[index, j], Predicted=p[index, j], Error=p[index, j] - y[index, j]))
    # Load-level holdouts drive model selection. Constant offsets are comparison only for a tareable model.
    choices = [e for e in evaluations if e["Validation"] == "leave_load_out"
               and e["Samples"] == len(known) and e["Model"] != "intercept"]
    method = min(choices, key=lambda e: e["Normalized_Joint_RMSE"])["Model"] if choices else "OLS"
    c, bias = fit(x, y, method, noise)
    regression = np.linalg.lstsq(y / refs, x, rcond=None)[0]
    signal = np.linalg.svd(regression.T, compute_uv=False)

    # Resample repeat labels as clusters. Diagnostic variation is not a metrological uncertainty interval.
    generator = np.random.default_rng(80)
    repeat_groups = np.array(splits["leave_repeat_out"])
    ids = np.unique(repeat_groups)
    estimates = []
    for _ in range(200):
        picked = np.concatenate([np.flatnonzero(repeat_groups == k) for k in generator.choice(ids, size=len(ids))])
        estimates.append(fit(x[picked], y[picked], method, noise[picked])[0])
    return dict(method=method, active=names, indexes=active, coefficients=c, intercept=bias,
                measurements=known, evaluations=evaluations, predictions=predictions,
                physical=svd_info(y / refs), raw=svd_info(x),
                forward=svd_info(regression.T), forward_singular_values=signal.tolist(),
                coefficient_sd=np.std(estimates, axis=0, ddof=1),
                observed_limits={a: [float(y[:, j].min()), float(y[:, j].max())] for j, a in enumerate(names)})

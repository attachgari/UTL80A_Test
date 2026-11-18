"""RAW28 analysis, paired hold measurements, and exploratory torque events."""

from collections import defaultdict
from itertools import groupby

import numpy as np

from calibration_data import CHANNELS, TEMPS, number, pairs, quality
from calibration_model import AXES, reference, vec
from tz_click_peaks import add_clicks


def provenance(run):
    return dict(Source_File=run.source, Sheet_Name=run.sheet, Target_Axis=run.axis,
                Applied_Load=run.load, Load_Unit=run.unit)


def audit(log, run, rows, issues):
    for row, reasons in zip(rows, issues):
        if reasons:
            log.append(dict(**provenance(run), Excel_Row=row["source_row"], Time=row["Time"],
                            Repeat=row.get("Repeat"), State=row.get("State"),
                            Excluded=any(block for _, block in reasons),
                            Reasons=";".join(reason for reason, _ in reasons)))


def inspect_window(times, raw, keep):
    available = np.flatnonzero(keep)
    if len(available) < 12:
        return available, "insufficient"
    t, x = times[available], raw[available]
    if t[-1] - t[0] < 3:
        return available, "capture_under_3s"
    thirds = np.array_split(np.arange(len(x)), 3)
    means = np.array([x[split].mean(axis=0) for split in thirds])
    # Within-segment RMS provides a conservative bound for detecting a sustained level shift.
    bounds = 2 * np.maximum(np.median([x[s].std(axis=0, ddof=1) for s in thirds], axis=0), 1)
    late = np.abs(means[2] - means[1]) > bounds
    early = np.abs(means[0] - means[2]) > bounds
    if np.any(late):
        return available, "late_level_shift_review"
    if np.any(early):
        return available[np.concatenate(thirds[1:])], "last_two_thirds_after_early_level_shift"
    return available, "full_segment"


def step_run(run, geometry, qc, step_stats, experiments, torque, traces, click_peaks=0, peak_traces=None):
    is_tz = run.kind == "tz_step"
    required = "Torque" if is_tz else "Loaded"
    for pair, error, urows, lrows in pairs(run, required):
        if error:
            qc.append(dict(**provenance(run), Excel_Row=(urows or lrows)[0]["source_row"],
                           Excluded=True, Reasons=error))
            continue
        batch, repeat = pair
        identity = f"{run.key}::{batch}:{repeat}"
        base = dict(**provenance(run), Test_ID=identity, Batch=batch, Repeat=repeat)
        tu, u, ut, uk, ui = quality(urows, True)
        tl, l, lt, lk, li = quality(lrows, True)
        audit(qc, run, urows, ui)
        audit(qc, run, lrows, li)
        # Use the recorded first five seconds for the click baseline. Ready samples remain a diagnostic.
        baseline_keep = uk & (np.array([number(r["Elapsed_Time"]) for r in urows]) < 5) if is_tz else uk
        wu, result_u = inspect_window(tu, u, baseline_keep)
        wl, result_l = inspect_window(tl, l, lk) if not is_tz else (np.flatnonzero(lk), "dynamic_torque")
        if not len(wu) or not len(wl):
            qc.append(dict(**provenance(run), Excel_Row=urows[0]["source_row"], Repeat=repeat,
                           Excluded=True, Reasons="no_valid_paired_samples"))
            continue
        u0 = u[wu].mean(axis=0)
        if is_tz:
            if click_peaks:
                add_clicks(run, base, tl, l, lt, wl, lrows, u0, u[wu], ut[wu], click_peaks,
                           torque, experiments, peak_traces)
                trace_peaks = []
                dynamic_torque(run, base, urows, tu, u, uk, tl, l, wl, lrows, u0, wu, trace_peaks, traces)
                continue
            dynamic_torque(run, base, urows, tu, u, uk, tl, l, wl, lrows, u0, wu, torque, traces)
            config = geometry.get(run.key, {}).get("torque_references", {}).get(f"{batch}:{repeat}")
            if config is None:
                experiments.append(dict(**base, Load_Position="", Force_Direction="", Force_Application_Point_m="",
                                        **dict.fromkeys(AXES), **{f"Delta_{ch}": None for ch in CHANNELS},
                                        **dict(zip(TEMPS, lt[wl].mean(axis=0))),
                                        **{f"Unloaded_{ch}": x for ch, x in zip(TEMPS, ut[wu].mean(axis=0))},
                                        Data_Quality="dynamic torque; see tz_peak_candidates.csv",
                                        Reference_Evidence="click wrench setting without synchronized reference",
                                        Calibration_Eligible=False))
                continue
            if not config.get("evidence"):
                raise ValueError(f"{identity}: synchronized torque reference needs evidence")
            first, last = config["stable_excel_rows"]
            wl = wl[[first <= lrows[i]["source_row"] <= last for i in wl]]
            if len(wl) < 12:
                raise ValueError(f"{identity}: fewer than 12 valid samples in synchronized reference interval")
            result_l = "synchronized_reference_interval"
        loaded = l[wl].mean(axis=0)
        delta = loaded - u0
        t_load = lt[wl].mean(axis=0)
        t_unload = ut[wu].mean(axis=0)
        sd_u = u[wu].std(axis=0, ddof=1) if len(wu) > 1 else np.full(8, np.nan)
        sd_l = l[wl].std(axis=0, ddof=1) if len(wl) > 1 else np.full(8, np.nan)
        if is_tz:
            wrench, position, direction, point, evidence = vec(config["wrench"], 6), "", None, None, config["evidence"]
            torque[-1]["Calibration_Eligible"] = True
            torque[-1]["Reason"] = "separate externally synchronized interval; peak itself remains exploratory"
        else:
            wrench, position, direction, point, evidence = reference(run, geometry)
        suitable = all(result not in {"insufficient", "capture_under_3s", "late_level_shift_review"}
                       for result in (result_u, result_l))
        variances = sd_u**2 / len(wu) + sd_l**2 / len(wl)
        for j, channel in enumerate(CHANNELS):
            step_stats.append(dict(**base, Channel=channel, Unloaded_Mean=u0[j], Loaded_Mean=loaded[j],
                                   Delta=delta[j], Unloaded_SD=sd_u[j], Loaded_SD=sd_l[j],
                                   Unloaded_N=len(wu), Loaded_N=len(wl), Unloaded_Valid=int(uk.sum()),
                                   Loaded_Valid=int(lk.sum()), Unloaded_Start_s=tu[wu[0]], Unloaded_End_s=tu[wu[-1]],
                                   Loaded_Start_s=tl[wl[0]], Loaded_End_s=tl[wl[-1]],
                                   Unloaded_First_Excel_Row=urows[wu[0]]["source_row"],
                                   Unloaded_Last_Excel_Row=urows[wu[-1]]["source_row"],
                                   Loaded_First_Excel_Row=lrows[wl[0]]["source_row"],
                                   Loaded_Last_Excel_Row=lrows[wl[-1]]["source_row"],
                                   Unloaded_Window=result_u, Loaded_Window=result_l))
        experiments.append(dict(**base, Load_Position=position, Force_Direction=format_vec(direction),
                                Force_Application_Point_m=format_vec(point),
                                **dict(zip(AXES, wrench if wrench is not None else [None]*6)),
                                **{f"Delta_{name}": val for name, val in zip(CHANNELS, delta)},
                                **dict(zip(TEMPS, t_load)), **{f"Unloaded_{name}": val for name, val in zip(TEMPS, t_unload)},
                                Data_Quality=f"{result_u}/{result_l}; valid {uk.sum()}/{len(u)} + {lk.sum()}/{len(l)}",
                                Reference_Evidence=evidence, Calibration_Eligible=bool(wrench is not None and suitable),
                                Delta_Mean_Variance=float(np.mean(variances)), Zero_Vector=u0))
        if not is_tz:
            traces.append((f"{run.axis} {run.load:g} kg • B{batch} R{repeat}", [(tu[uk], u[uk], "-"),
                           (tl[lk] + 5, l[lk], "--")]))


def format_vec(values):
    return "" if values is None else ",".join(f"{v:.8g}" for v in values)


def dynamic_torque(run, base, urows, tu, u, uk, tl, loaded, wl, lrows, u0, wu, torque, traces):
    x = loaded[wl] - u0
    amplitude = np.linalg.norm(x, axis=1)
    # Median in a short moving window suppresses an isolated single-sample excursion.
    k = min(5, len(wl))
    smoothed = np.array([np.median(amplitude[max(0, i-k//2):i+k//2+1]) for i in range(len(wl))])
    peak = int(np.argmax(smoothed))
    end = np.array([number(r["Elapsed_Time"]) for r in urows]) >= 5
    ready = np.linalg.norm(u[uk & end] - u0, axis=1)
    score = np.linalg.norm(u[wu] - u0, axis=1)
    row = dict(**base, Unit_of_Peak="RAW28 counts (8-channel norm)", Peak_Norm=amplitude[peak],
               Smoothed_Peak_Norm=smoothed[peak], Peak_s_from_marker=tl[wl[peak]],
               Candidate_From_s=tl[wl[max(0, peak-k//2)]], Candidate_To_s=tl[wl[min(len(wl)-1, peak+k//2)]],
               Candidate_Excel_Row=lrows[wl[peak]]["source_row"], Baseline_P95=np.quantile(score, .95),
               Ready_Max=float(ready.max()) if ready.size else None, Ready_Samples=len(ready),
               Peak_At_Boundary=peak < k//2 or peak >= len(wl)-k//2,
               Calibration_Eligible=False, Reason="click setting, no synchronized reference or Enter-only control")
    row.update({f"Peak_Delta_{ch}": value for ch, value in zip(CHANNELS, x[peak])})
    torque.append(row)
    offset = next(number(r["Elapsed_Time"]) for r in lrows if np.isfinite(number(r["Elapsed_Time"]))) - next(
        number(r["Elapsed_Time"]) for r in urows if np.isfinite(number(r["Elapsed_Time"])))
    traces.append((f"{run.axis} {run.load:g} N m • B{base['Batch']} R{base['Repeat']}",
                   [(tu[uk], u[uk]-u0, "-"), (tl[wl]+offset, x, "--")]))


def summarize_steps(stats):
    groups = defaultdict(list)
    for row in stats:
        key = tuple(row[k] for k in ("Source_File", "Sheet_Name", "Target_Axis", "Applied_Load", "Batch", "Channel"))
        groups[key].append(row)
    output = []
    for (source, sheet, axis, mass, batch, channel), rows in groups.items():
        mean = float(np.mean([r["Delta"] for r in rows]))
        sd = float(np.std([r["Delta"] for r in rows], ddof=1)) if len(rows) > 1 else np.nan
        zeros = [r["Unloaded_Mean"] for r in rows]
        output.append(dict(Source_File=source, Sheet_Name=sheet, Target_Axis=axis, Applied_Load=mass, Batch=batch,
                           Channel=channel, Repeats=len(rows), Delta_Mean=mean, Delta_SD=sd,
                           Relative_Repeatability_Pct=100 * sd / abs(mean) if mean else None,
                           Unloaded_Zero_Peak_to_Peak=np.ptp(zeros), Unloaded_Zero_SD=np.std(zeros, ddof=1)
                           if len(zeros) > 1 else None))
    return output


def symmetry(experiments):
    groups = defaultdict(list)
    for row in experiments:
        if row["Delta_L1"] is None or row["Load_Unit"] != "kg":
            continue
        groups[row["Target_Axis"], row["Applied_Load"]].append([row[f"Delta_{ch}"] for ch in CHANNELS])
    averages = {key: np.mean(values, axis=0) for key, values in groups.items()}
    result = []
    unit_response = {}
    for mass in sorted({k[1] for k in averages}):
        shared = {}
        for axis in ("Fx", "Fy", "Tx", "Ty"):
            p, m = averages.get((axis+"+", mass)), averages.get((axis+"-", mass))
            if p is None or m is None:
                continue
            common, differential = (p+m)/2, (p-m)/2
            denom = np.linalg.norm(p)*np.linalg.norm(m)
            cosine = float(-p @ m / denom) if denom else None
            unit = differential / mass if mass else np.full(8, np.nan)
            expected = unit_response.setdefault(axis, unit)
            ratio = np.linalg.norm(expected)
            change = np.linalg.norm(unit-expected)/ratio if ratio else None
            row = dict(Axis=axis, Mass_kg=mass, Cosine_Pos_vs_Neg_Inverted=cosine,
                       Common_Norm=np.linalg.norm(common), Differential_Norm=np.linalg.norm(differential),
                       Norm_Unit_Response_Change_vs_Lowest_Pct=None if change is None else 100*change)
            row.update({f"Common_{c}": v for c, v in zip(CHANNELS, common)})
            row.update({f"Differential_{c}": v for c, v in zip(CHANNELS, differential)})
            result.append(row)
            if axis in ("Tx", "Ty"):
                shared[axis] = common
        if len(shared) == 2:
            denom = np.linalg.norm(shared["Ty"])
            result.append(dict(Axis="Txy_common_comparison", Mass_kg=mass,
                               AX_vs_AY_Relative_L2=np.linalg.norm(shared["Ty"]-shared["Tx"])/denom if denom else None))
    return result


def correlation(a, b):
    return float(np.corrcoef(a, b)[0, 1]) if np.std(a) and np.std(b) else None


def hold_run(run, qc, results, stability, temperature, psd):
    t, raw, temp, mask, flags = quality(run.rows, False)
    audit(qc, run, run.rows, flags)
    t, raw, temp = t[mask], raw[mask], temp[mask]
    if len(t) < 20 or np.ptp(t) <= 0:
        return t, raw, temp
    design = np.c_[t, np.ones(len(t))]
    slopes = np.linalg.lstsq(design, raw, rcond=None)[0]
    detrended = raw - design @ slopes
    t_detrended = temp - design @ np.linalg.lstsq(design, temp, rcond=None)[0]
    ptp = np.ptp(t)
    dt = np.diff(t)
    dt50 = np.median(dt)
    jitter = np.quantile(np.abs(dt-dt50), 0.95)/dt50 if dt50 > 0 else np.inf
    # Average observed samples into 1 s bins, preserving empty bins and time order.
    seconds = np.floor(t-t[0]).astype(int)
    bins = np.full((seconds.max()+1, 8), np.nan)
    for second, group in groupby(np.arange(len(t)), lambda i: seconds[i]):
        bins[second] = detrended[list(group)].mean(axis=0)
    psd_ok = len(t) >= 2048 and jitter <= .2 and np.max(dt) <= 3*dt50 and np.all(dt > 0)
    if psd_ok:
        psd.extend(welch(run, t, detrended, dt50))
    for j, channel in enumerate(CHANNELS):
        x, tx = raw[:, j], temp[:, j//2]
        results.append(dict(**provenance(run), Channel=channel, Samples=len(t), Total=len(run.rows),
                            Span_s=ptp, Arrival_Rate_SPS=(len(t)-1)/ptp, Receive_Median_DT_s=dt50,
                            Receive_Relative_Jitter_P95=jitter, Mean=x.mean(), Median=np.median(x),
                            SD=x.std(ddof=1), Detrended_RMS_Noise=np.sqrt(np.mean(detrended[:, j]**2)),
                            Peak_to_Peak=np.ptp(x), Minimum=x.min(), Maximum=x.max(), RAW_per_s=slopes[0, j],
                            RAW_per_min=60*slopes[0, j], Last_30s_minus_First_30s=x[t>=t[-1]-30].mean()-x[t<t[0]+30].mean(),
                            Coil_Temperature=TEMPS[j//2], Temperature_Min=tx.min(), Temperature_Max=tx.max(),
                            Temperature_Correlation=correlation(x, tx), Temperature_Correlation_Control_Time=
                            correlation(detrended[:, j], t_detrended[:, j//2]),
                            PSD_Status="receive-time Welch estimate" if psd_ok else "skipped: timing quality gate"))
        for tau in (1, 2, 5, 10, 20, 30, 60):
            b = bins[:len(bins)//tau*tau, j].reshape(-1, tau).mean(axis=1)
            d = np.diff(b)
            d = d[np.isfinite(d)]
            if len(d) >= 2:
                stability.append(dict(**provenance(run), Channel=channel, Tau_s=tau, Pairs=len(d),
                                      Allan_Deviation_RAW=float(np.sqrt(np.mean(d**2)/2)),
                                      Method="nonoverlapping observed 1 s receive bins, linear drift removed"))
    temperature.append((run.key, run.axis, run.load, detrended, t_detrended))
    return t, raw, temp


def welch(run, t, detrended, period):
    size = 1024
    grid = np.arange(t[0], t[-1], period)
    values = np.stack([np.interp(grid, t, col) for col in detrended.T], axis=1)
    window = np.hanning(size)
    frequencies = np.fft.rfftfreq(size, period)
    powers = []
    for start in range(0, len(values)-size+1, size//2):
        x = values[start:start+size]
        transform = np.fft.rfft((x-x.mean(axis=0))*window[:, None], axis=0)
        power = abs(transform)**2 * period / np.sum(window**2)
        power[1:-1] *= 2
        powers.append(power)
    average = np.mean(powers, axis=0)
    return [dict(**provenance(run), Channel=ch, Frequency_Hz=f, PSD_counts2_per_Hz=average[i, j])
            for i, f in enumerate(frequencies) for j, ch in enumerate(CHANNELS)]


def thermal_comparison(runs):
    result = []
    groups = defaultdict(list)
    for key, axis, load, raw, temp in runs:
        groups[axis, load].append((key, raw, temp))
    for j, ch in enumerate(CHANNELS):
        for name in ("none", "coil_linear", "coil_and_tmp275", "all_linear", "coil_quadratic"):
            errors = []
            for (axis, load), datasets in groups.items():
                # Same direction and load are required so the holdout cannot learn a load offset.
                if len(datasets) < 2:
                    continue
                for i, (_, raw, temp) in enumerate(datasets):
                    train = [d for k, d in enumerate(datasets) if k != i]
                    def features(value):
                        coil = value[:, j//2]
                        return {"coil_linear": coil[:, None], "coil_and_tmp275": value[:, [j//2, 4]],
                                "all_linear": value, "coil_quadratic": np.c_[coil, coil**2]}[name]
                    estimate = np.zeros(len(raw))
                    if name != "none":
                        x = np.vstack([features(d[2]) for d in train])
                        y = np.concatenate([d[1][:, j] for d in train])
                        if np.linalg.matrix_rank(x) < x.shape[1]:
                            continue
                        b = np.linalg.lstsq(x, y, rcond=None)[0]
                        estimate = features(temp) @ b
                    errors.append(float(np.mean((raw[:, j]-estimate)**2)))
            result.append(dict(Channel=ch, Model=name, Matched_Independent_Run_Holdouts=len(errors),
                               Detrended_RMSE_RAW=np.sqrt(np.mean(errors)) if errors else None,
                               Deployable=False, Reason="requires same-load independent controlled thermal runs"))
    return result

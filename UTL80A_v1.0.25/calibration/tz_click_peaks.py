"""Extract one synchronous RAW28 maximum per nominal click-wrench event."""

import numpy as np

from calibration_data import CHANNELS, TEMPS
from calibration_model import AXES


def select_peaks(t, delta, baseline, expected, separation_s=.75):
    amplitude = np.linalg.norm(delta, axis=1)
    smooth = np.median(np.lib.stride_tricks.sliding_window_view(np.pad(amplitude, 2, mode="edge"), 5), axis=1)
    floor = 4 * float(np.quantile(np.linalg.norm(baseline, axis=1), .95))
    candidates = np.flatnonzero((smooth[1:-1] >= smooth[:-2]) & (smooth[1:-1] > smooth[2:])) + 1
    selected = []
    for i in sorted(candidates, key=lambda i: smooth[i], reverse=True):
        if smooth[i] >= floor and all(abs(t[i] - t[j]) >= separation_s for j in selected):
            selected.append(i)
    detected = len(selected)
    if detected < expected:
        raise ValueError(f"only {detected} well-separated peaks passed the noise gate (expected {expected})")
    event_centers = sorted(selected[:expected])
    peaks = []
    for center in event_centers:
        near = np.flatnonzero(abs(t - t[center]) <= .1)
        peak = near[np.argmax(amplitude[near])]
        peaks.append((peak, center))
    return peaks, amplitude, smooth, floor, detected


def add_clicks(run, base, tl, loaded, temps, wl, lrows, u0, unloaded, unloaded_temps,
               expected, torque, experiments, peak_traces):
    t, delta = tl[wl], loaded[wl] - u0
    try:
        peaks, amplitude, smooth, floor, detected = select_peaks(t, delta, unloaded-u0, expected)
    except ValueError as error:
        raise ValueError(f"{base['Test_ID']}: {error}") from error
    signed_setting = run.load if run.axis == "Tz+" else -run.load
    wrench = dict.fromkeys(AXES, 0.0)
    wrench["Tz"] = signed_setting
    evidence = "Nominal click setting assigned to each selected event; other wrench components assumed zero"
    for event, (i, center) in enumerate(peaks, 1):
        excel = lrows[wl[i]]
        row = dict(**{**base, "Test_ID": f"{base['Test_ID']}:peak{event}"}, Peak_Number=event,
                   Peak_s_from_marker=t[i], Detection_Center_s=t[center],
                   Candidate_Excel_Row=excel["source_row"], Time=excel["Time"], Peak_Norm=amplitude[i],
                   Smoothed_Peak_Norm=smooth[center], Detection_Floor_Norm=floor,
                   Candidates_Above_Floor=detected, Tz_Nominal_Nm=signed_setting,
                   Calibration_Eligible=True, Reason=evidence)
        row.update({f"Peak_Delta_{ch}": val for ch, val in zip(CHANNELS, delta[i])})
        torque.append(row)
        experiments.append(dict(**{**base, "Test_ID": row["Test_ID"]}, Load_Position="", Force_Direction="",
                                Force_Application_Point_m="", **wrench,
                                **{f"Delta_{ch}": val for ch, val in zip(CHANNELS, delta[i])},
                                **dict(zip(TEMPS, temps[wl[i]])),
                                **{f"Unloaded_{ch}": val for ch, val in zip(TEMPS, unloaded_temps.mean(axis=0))},
                                Data_Quality="nominal click peak; synchronized eight-channel RAW28 sample",
                                Peak_Number=event, Excel_Row=excel["source_row"], Time=excel["Time"],
                                Reference_Evidence=evidence, Calibration_Eligible=True,
                                Delta_Mean_Variance=float(unloaded.var(axis=0, ddof=1).mean()),
                                Zero_Vector=u0))
    peak_traces.append((run.sheet, base["Repeat"], t, smooth, amplitude, [i for i, _ in peaks]))

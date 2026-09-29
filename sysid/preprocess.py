"""Signal conditioning shared by synthetic and hardware data.

The same zero-phase low-pass is applied to the wrench and (through the Savitzky-Golay
derivatives) to the joint signals, so model and measurement see the same filtering.
"""
import numpy as np
from scipy.signal import butter, filtfilt, savgol_filter


def lowpass(x, fs, fc, order=4):
    b, a = butter(order, fc / (0.5 * fs))
    return filtfilt(b, a, x, axis=0)


def derivatives(q, fs, window_s=0.08, poly=3):
    """Smoothed q, qdot, qddot from uniformly sampled positions (Savitzky-Golay)."""
    n = max(poly + 2, int(round(window_s * fs)) | 1)
    dt = 1.0 / fs
    return (savgol_filter(q, n, poly, axis=0),
            savgol_filter(q, n, poly, deriv=1, delta=dt, axis=0),
            savgol_filter(q, n, poly, deriv=2, delta=dt, axis=0))


def resample(t_src, x_src, t_dst):
    """Linear interpolation of each column of x_src onto t_dst."""
    x_src = np.asarray(x_src)
    if x_src.ndim == 1:
        return np.interp(t_dst, t_src, x_src)
    return np.column_stack([np.interp(t_dst, t_src, x_src[:, i]) for i in range(x_src.shape[1])])

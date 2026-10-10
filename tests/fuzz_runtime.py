"""Shared wall-time and batch sizing for the extended fuzz campaigns."""

import math
import os
import time


def budget():
    """Return a monotonic start/deadline for each independently timed campaign."""
    minutes = float(os.environ.get('PIPESIM_FUZZ_MINUTES', '15'))
    if not math.isfinite(minutes) or minutes <= 0:
        raise ValueError('PIPESIM_FUZZ_MINUTES must be a positive finite number')
    started = time.monotonic()
    return minutes, started, started + minutes * 60


def batch_size(maximum, remaining_cases, completed, started, deadline, minutes):
    """Keep initial and final batches proportional to the available time."""
    if completed:
        seconds_per_case = (time.monotonic() - started) / completed
        time_limited = max(1, int((deadline - time.monotonic()) / seconds_per_case))
    else:
        time_limited = max(1, math.ceil(maximum * min(minutes, 10) / 10))
    return min(maximum, remaining_cases, time_limited)

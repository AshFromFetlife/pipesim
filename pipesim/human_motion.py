"""Engineering joint envelopes and deterministic activity, not clinical models.

The presets deliberately describe test envelopes. They do not predict population
percentiles, diagnoses, fatigue, tissue damage, or an individual's safe motion.
"""
from functools import lru_cache
import hashlib
import math

import numpy as np

FLEXIBILITIES = ('minimum', 'athletic', 'gymnast', 'contortionist',
                 'full_socket_span', 'fragile', 'full_360')
ACTIVITIES = ('random_spasms', 'fidget', 'struggle', 'destructive')
POSTURES = ('passive', 'hold', 'arms', 'torso', 'upper_body', 'legs', *ACTIVITIES)


def capacity_nm(name, mass_kg=75, strength_scale=1):
    name = name.rsplit('/', 1)[-1]
    capacity = (120 if name.endswith('_hip') else 90 if name.endswith('_knee') else
                40 if name.endswith('_ankle') else 35 if name.endswith('_shoulder') else
                20 if name.endswith('_elbow') else 5 if name.endswith('_wrist') else
                10 if 'neck' in name else 60)
    return capacity * strength_scale * mass_kg / 75


def joint_envelope(name, kind, limits, flexibility):
    """Return neutral limits; broad modes retain physical segment contact."""
    if flexibility in ('full_socket_span', 'full_360'):
        return 'spherical', {'rotation_deg': [[-1e8, 1e8] for _ in range(3)]}
    base = flexibility if flexibility != 'fragile' else 'minimum'
    ranges = [limits['angle_deg']] if kind == 'revolute' else limits['rotation_deg']
    factor = {'minimum': .78, 'athletic': 1, 'gymnast': 1.35, 'contortionist': 1.8}[base]
    result = [[round(lo*factor, 4), round(hi*factor, 4)] for lo, hi in ranges]
    if base in ('gymnast', 'contortionist'):
        hyper = 15 if base == 'gymnast' else 35
        if name.endswith('_hip'):
            result = [[-100 if base == 'gymnast' else -150, 165 if base == 'gymnast' else 200],
                      [-100 if base == 'gymnast' else -145, 100 if base == 'gymnast' else 145],
                      [-90 if base == 'gymnast' else -130, 90 if base == 'gymnast' else 130]]
        if kind == 'revolute':
            result[0] = [-hyper, 160 if base == 'gymnast' else 175] if name.endswith('_elbow') else [-165 if base == 'gymnast' else -180, hyper]
            return 'spherical', {'rotation_deg': [result[0], [-hyper/2, hyper/2], [-hyper, hyper]]}
    if flexibility == 'fragile' and kind == 'revolute':
        # Preallocate the extra axes so releasing a joint preserves its pivot
        # and all velocity coordinates. The transverse stops are initially tight.
        return 'spherical', {'rotation_deg': [result[0], [-.1, .1], [-.1, .1]]}
    return kind, {'angle_deg': result[0]} if kind == 'revolute' else {'rotation_deg': result}


@lru_cache(maxsize=8192)
def _sample(seed, joint, interval):
    token = hashlib.blake2b(f'{seed}:{joint}:{interval}'.encode(), digest_size=16).digest()
    rng = np.random.default_rng(int.from_bytes(token, 'little'))
    vector = rng.uniform(-1, 1, 3)
    vector /= max(float(np.linalg.norm(vector)), 1e-12)
    return tuple(vector), float(rng.uniform(.55, 1)), float(rng.uniform(.02, .48))


def activity_torque(mode, seed, joint, time_s, capacity):
    """Joint-local torque vector, deterministic at a time regardless of timestep.

    Fidget uses 4% of peak strength, struggle 18%; both are explicit engineering
    assumptions. Bursts use up to peak strength, never an unbounded impulse.
    """
    period = {'fidget': 1., 'struggle': .7, 'random_spasms': .65, 'destructive': .22}[mode]
    interval = int(math.floor(max(0., time_s) / period))
    vector, amplitude, start = _sample(seed, joint, interval)
    phase = max(0., time_s) / period - interval
    if mode == 'fidget': amount = .04 * amplitude
    elif mode == 'struggle':
        previous, old_amplitude, _ = _sample(seed, joint, max(0, interval-1))
        blend = phase * phase * (3-2*phase)
        return capacity*.18*((1-blend)*old_amplitude*np.array(previous)+blend*amplitude*np.array(vector))
    elif mode == 'random_spasms': amount = amplitude if start <= phase < start + .13 else 0.
    else: amount = 1. if phase < .78 else 0.
    return np.array(vector) * (capacity * amount)


def same_human_no_collision(a, b):
    """Full 360 disables only this mannequin's self contact, never world contact."""
    return (a.kind == b.kind == 'human' and a.id.rsplit('/', 1)[0] == b.id.rsplit('/', 1)[0]
            and a.definition.get('source', {}).get('flexibility') == 'full_360'
            and b.definition.get('source', {}).get('flexibility') == 'full_360')

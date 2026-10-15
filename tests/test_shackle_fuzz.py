"""Seeded drag-style shackle connections across poses and the saved legacy ID."""

import random

import numpy as np

from pipesim.document import Assembly
from pipesim.port_connections import connect_ports
from pipesim.validation import validate


def test_seeded_shackle_hook_connections_have_no_false_contact_warnings():
    rng = random.Random(451897)
    for catalog in ('generic.d-shackle', 'generic.d-link'):
        for seed in range(12):
            pose = {'position_mm': [rng.uniform(-120, 120), rng.uniform(-120, 120),
                                    rng.uniform(100, 300)],
                    'rotation_deg': [rng.uniform(-35, 35), rng.uniform(-35, 35),
                                     rng.uniform(-180, 180)]}
            doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
                   'name': 'Shackle hook fuzz',
                   'parts': [{'id': 'shackle', 'catalog': catalog, 'pose': pose},
                             {'id': 'link', 'catalog': 'generic.chain-link',
                              'pose': {'position_mm': [0, 0, 200]}}],
                   'joints': []}
            result = connect_ports(Assembly.from_doc(doc),
                                   {'part': 'shackle', 'port': 'eye'},
                                   {'part': 'link', 'port': 'a'},
                                   'spherical', move='a')
            connected = Assembly.from_doc(result['document'])
            a, b, _ = connected.joint_frames(result['joint'])
            assert np.linalg.norm(a - b) < 1e-6, (catalog, seed)
            issues = validate(connected)['issues']
            assert not any(item['code'] in ('INTERSECTION', 'UNREACHABLE_JOINT')
                           for item in issues), (catalog, seed, issues)


def test_seeded_shackle_pin_connections_reach_hinge_eyes_and_bolt_holes():
    rng = random.Random(832031)
    for catalog, port in (('tubeclamp.TC173MC', 'hinge'),
                          ('tubeclamp.TC131T', 'bolt1')):
        for seed in range(8):
            pose = {'position_mm': [rng.uniform(-120, 120), rng.uniform(-120, 120),
                                    rng.uniform(100, 300)],
                    'rotation_deg': [rng.uniform(-180, 180) for _ in range(3)]}
            doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
                   'name': 'Shackle pin fuzz',
                   'parts': [{'id': 'shackle', 'catalog': 'generic.d-shackle', 'pose': pose},
                             {'id': 'target', 'catalog': catalog,
                              'pose': {'position_mm': [0, 0, 200]}}],
                   'joints': []}
            result = connect_ports(Assembly.from_doc(doc),
                                   {'part': 'shackle', 'port': 'pin_bolt'},
                                   {'part': 'target', 'port': port},
                                   'revolute', move='a')
            connected = Assembly.from_doc(result['document'])
            a, b, _ = connected.joint_frames(result['joint'])
            assert np.linalg.norm(a - b) < 1e-6, (catalog, seed)
            issues = validate(connected)['issues']
            assert not any(item['code'] == 'UNREACHABLE_JOINT' for item in issues), (catalog, seed, issues)

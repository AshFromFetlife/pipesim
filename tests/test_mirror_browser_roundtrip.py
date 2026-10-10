"""Mirror reopen must survive the JSON number representation used by browsers."""
import copy
import json

from pipesim.drafting import _mirror_reopen_digest, finalize, remember_mirrored_draft, reopen
from pipesim.symmetry import materialize_all


def test_mirror_reopen_survives_browser_integer_numbers(factory, blank):
    source = copy.deepcopy(blank)
    source['draft_subassemblies'] = [{
        'id': 'frame', 'runs': [{'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
            'start_mm': [-0.0, 200.0, 100.25], 'end_mm': [0.0, 200.0, 900.25],
            'attachments': []}],
        'mirrors': [{'id': 'side', 'scope': 'scene', 'axis': 'y', 'offset_mm': 0.0}]}]
    baked = materialize_all(factory(source))
    result = remember_mirrored_draft(source, finalize(factory(baked)))
    assert result['status'] == 'finalized'
    # Preserve existing records written before numeric normalization, e.g. an
    # unchanged YAML document that still has its original integral floats.
    legacy = copy.deepcopy(result['document'])
    legacy['metadata']['draft_mirror_reopen']['final_digest'] = _mirror_reopen_digest(
        legacy, normalize_numbers=False)
    assert reopen(factory(legacy), ['pipe']).get('restored_mirrors') == 1
    # JSON.parse / JSON.stringify has one numeric type, so integral floats
    # return as integer tokens. Preserve non-integral values exactly.
    def browser(value):
        if isinstance(value, float) and value.is_integer():
            return int(value)
        if isinstance(value, list):
            return [browser(item) for item in value]
        if isinstance(value, dict):
            return {key: browser(item) for key, item in value.items()}
        return value
    returned = json.loads(json.dumps(browser(result['document'])))
    reopened = reopen(factory(returned), ['pipe'])
    assert reopened.get('restored_mirrors') == 1
    assert reopened['document'] == source
    # An actual geometry change must still invalidate the remembered source.
    changed = copy.deepcopy(returned)
    changed['parts'][0]['pose']['position_mm'][0] += 10
    ordinary = reopen(factory(changed), ['pipe'])
    assert not ordinary.get('restored_mirrors')

import copy

import pytest

from pipesim.document import DocumentError
from pipesim.physics import World


def test_limited_hinge_loop_has_different_guidance(factory,blank):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':pid,'catalog':'generic.box','label':pid.upper()} for pid in ('a','b','c')]
    doc['joints']=[{'id':a+'-'+b,'type':'revolute','a':{'part':a},'b':{'part':b},'limits':{'angle_deg':[-30,30]}} for a,b in [('a','b'),('b','c'),('c','a')]]
    with pytest.raises(DocumentError) as error: World(factory(doc))
    text=str(error.value)
    assert 'motor or travel limits' in text and 'preserving all its motors and limits' in text
    assert 'unlocked socket' not in text
    assert 'Connection: c-a' in text and '"C"' in text
    assert 'tree joint' not in text and 'gravity has not been applied' in text

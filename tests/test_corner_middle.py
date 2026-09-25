"""TC116 must accept a continuous upright and two separate perpendicular rails."""
import copy

import numpy as np
import pytest

from pipesim.document import DocumentError
from pipesim.editing import connect_member
from pipesim.physics import simulate
from pipesim.validation import validate


def corner_design(blank, size='C'):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'corner','catalog':f'tubeclamp.TC116{size}','pose':{'position_mm':[0,0,500]}}]
    doc['parts'] += [{'id':name,'catalog':f'tubeclamp.tube-{size}','parameters':{'length_mm':length}}
                     for name,length in [('upright',1000),('rail-x',400),('rail-y',400)]]
    doc['anchors']=[{'part':'upright','surface':'fixture'}]
    doc=connect_member(doc,'.','upright','corner','through',at_mm=500)
    for axis in ('x','y'):
        doc=connect_member(doc,'.','rail-'+axis,'corner',axis)
    return doc


@pytest.mark.parametrize('size,dimension,mass',[
    ('T',50.3,.166),('A',60.95,.258),('B',74.5,.39),('C',88.9,.634),('D',96.7,.688)
])
def test_corner_middle_three_pipe_fit(blank,factory,size,dimension,mass):
    assembly=factory(corner_design(blank,size))
    fitting=assembly.parts['corner']
    assert fitting.definition['drawing_dimensions_mm']=={'f':dimension}
    assert fitting.mass==pytest.approx(mass)
    assert set(fitting.ports)=={'through','x','y'}
    assert fitting.ports['through']['through']
    assert fitting.ports['through']['assembly']=='slide'
    assert all(not fitting.ports[p]['through'] for p in ('x','y'))
    # The rails must not plug the continuous bore or each other's openings.
    report=validate(assembly)
    assert report['valid'],report['issues']
    assert len(assembly.rigid_groups())==1
    directions=np.array([assembly.parts[p].matrix[:3,2] for p in ('upright','rail-x','rail-y')])
    assert np.allclose(directions@directions.T,np.eye(3))


def test_corner_through_is_one_socket_not_two_terminal_sockets(blank):
    doc=corner_design(blank)
    with pytest.raises(DocumentError,match='occupied'):
        connect_member(doc,'.','rail-x','corner','through',at_mm=200)


def test_loose_corner_and_attached_rails_slide_together(blank,factory):
    doc=corner_design(blank)
    locked=simulate(factory(doc),.2,10)
    assert locked['frames'][-1]['parts']['corner']['position_mm'][2]==pytest.approx(500,abs=.01)
    doc['joints'][0]['locked']=False
    assembly=factory(doc)
    assert {frozenset(group) for group in assembly.rigid_groups()}=={
        frozenset(['upright']),frozenset(['corner','rail-x','rail-y'])}
    motion=simulate(assembly,.2,10)
    start=motion['frames'][0]['parts']; end=motion['frames'][-1]['parts']
    assert end['upright']['position_mm']==pytest.approx(start['upright']['position_mm'],abs=.01)
    drop=start['corner']['position_mm'][2]-end['corner']['position_mm'][2]
    assert 100<drop<250
    for rail in ('rail-x','rail-y'):
        assert start[rail]['position_mm'][2]-end[rail]['position_mm'][2]==pytest.approx(drop,abs=1)

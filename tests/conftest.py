from pathlib import Path
import copy
import pytest
from pipesim.document import Assembly,Library,read

ROOT=Path(__file__).resolve().parents[1]
@pytest.fixture(scope='session')
def library(): return Library.load()
@pytest.fixture
def load(library):
    def loader(name): return Assembly.from_doc(read(ROOT/'examples'/f'{name}.pipe.yaml'),ROOT/'examples',library)
    return loader
@pytest.fixture
def factory(library):
    return lambda doc:Assembly.from_doc(doc,ROOT/'examples',library)
@pytest.fixture
def blank(): return {'format':'pipesim/1','units':'mm-kg-s-N-deg','name':'Test','parts':[],'joints':[],'anchors':[]}

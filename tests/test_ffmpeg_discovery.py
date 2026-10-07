from pathlib import Path
import pytest
from pipesim.document import DocumentError
from pipesim import rendering


def test_explicit_encoder_overrides_path(monkeypatch):
    monkeypatch.setenv('PIPESIM_FFMPEG','custom-encoder')
    monkeypatch.setattr(rendering.shutil,'which',lambda name:'chosen.exe' if name=='custom-encoder' else 'path.exe')
    assert rendering.find_ffmpeg()=='chosen.exe'


def test_invalid_override_explains_configuration(monkeypatch):
    monkeypatch.setenv('PIPESIM_FFMPEG','missing')
    monkeypatch.setattr(rendering.shutil,'which',lambda name:None)
    with pytest.raises(DocumentError,match='PIPESIM_FFMPEG'): rendering.find_ffmpeg()


def test_path_has_priority_over_git_bash_fallback(monkeypatch):
    monkeypatch.delenv('PIPESIM_FFMPEG',raising=False)
    monkeypatch.setattr(rendering.shutil,'which',lambda name:'path-encoder.exe')
    assert rendering.find_ffmpeg()=='path-encoder.exe'


@pytest.mark.skipif(rendering.os.name!='nt',reason='Windows shell-specific PATH fallback')
def test_git_bash_home_bin_is_found_without_process_path(monkeypatch,tmp_path):
    monkeypatch.delenv('PIPESIM_FFMPEG',raising=False)
    monkeypatch.setattr(rendering.shutil,'which',lambda name:None)
    monkeypatch.setattr(Path,'home',lambda:tmp_path)
    (tmp_path/'bin').mkdir(); encoder=tmp_path/'bin'/'ffmpeg.exe'; encoder.write_bytes(b'fixture')
    assert rendering.find_ffmpeg()==str(encoder)

import json
import subprocess
import sys
from pathlib import Path
import pytest
from pipesim.document import read,write,Assembly

ROOT=Path(__file__).resolve().parents[1]
def run(*args): return subprocess.run([sys.executable,'-m','pipesim',*map(str,args)],cwd=ROOT,capture_output=True,text=True,encoding='utf-8')

def test_cli_validation_and_record_roundtrip(tmp_path):
    target=tmp_path/'design.pipe.yaml'; write(target,read(ROOT/'examples/workbench.pipe.yaml'))
    result=run('validate',target,'--record','-o',tmp_path/'checks.json')
    assert result.returncode==0,result.stderr
    assert read(target)['results']['validate']['valid']
    assert read(tmp_path/'checks.json')['input_sha256']==Assembly.load(target).input_hash

def test_cli_generates_a_gripping_human_with_selective_muscles(tmp_path):
    path=tmp_path/'gripping.pipe.yaml'
    result=run('human','--pose','pull-up','--hold-joints','upper_body','--strength-scale',6,'--grip-diameter',42.4,'-o',path)
    assert result.returncode==0,result.stderr
    assembly=Assembly.load(path)
    assert len(assembly.parts)==19
    assert sum(bool(j.get('motor')) for j in assembly.joints)==12
    assert assembly.parts['left_hand'].definition['ports']['grip']['diameter_mm']==42.4

def test_cli_reports_invalid_input_without_traceback(tmp_path):
    target=tmp_path/'bad.pipe.yaml'; target.write_text('format: unknown\nparts: []')
    result=run('validate',target)
    assert result.returncode==2 and 'PipeSim:' in result.stderr
    assert 'Traceback' not in result.stderr


def test_cli_simulation_progress_and_grouping(tmp_path):
    target=tmp_path/'recording.json'
    result=run('simulate','examples/chain.pipe.yaml','--duration','.02','--chain-links-per-body',5,'-o',target)
    assert result.returncode==0,result.stderr
    assert '[simulation' in result.stdout
    recording=read(target)
    assert recording['chain_simplification']['links_per_body']==5
    assert recording['chain_simplification']['frozen_joints']
    result=run('simulate','examples/chain.pipe.yaml','--duration','.01','--quiet')
    assert result.returncode==0,result.stderr
    assert '[simulation' not in result.stdout+result.stderr
    assert json.loads(result.stdout)['frames']


def test_cli_interrupt_returns_130_without_writing_recording(tmp_path,monkeypatch,capsys):
    from pipesim.cli import main
    from pipesim import simulation_jobs
    def interrupt(*args,**kwargs): raise KeyboardInterrupt()
    monkeypatch.setattr(simulation_jobs,'simulate_in_worker',interrupt)
    target=tmp_path/'recording.json'
    assert main(['simulate','examples/chain.pipe.yaml','--record','-o',str(target)])==130
    assert not target.exists()
    assert 'cancelled' in capsys.readouterr().err

def test_cli_expected_failure_has_nonzero_exit_code(tmp_path):
    result=run('fit','examples/human.pipe.yaml','--human','person','--target',0,5000,1000,'-o',tmp_path/'reach.json')
    assert result.returncode==1
    assert not read(tmp_path/'reach.json')['reachable']

def test_render_camera_and_extension_arguments(tmp_path):
    path=tmp_path/'view.jpg'
    result=run('render','examples/material-comparison.pipe.yaml','--width',160,'--height',120,'--eye',1500,-1500,1500,'--target',250,350,800,'--orthographic','-o',path)
    assert result.returncode==0,result.stderr
    assert path.read_bytes()[:2]==b'\xff\xd8'

def test_expanded_articulated_object_keeps_library_paths_after_relocation(tmp_path):
    target=tmp_path/'caster.pipe.yaml'
    result=run('expand','examples/caster-platform.pipe.yaml','-o',target)
    assert result.returncode==0,result.stderr
    a=Assembly.load(target)
    assert len(a.parts)==13 and len(a.joints)==12

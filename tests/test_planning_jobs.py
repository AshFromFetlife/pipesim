import threading
import time
from types import SimpleNamespace

import pytest
from pipesim.planning_jobs import PlanningJobs


def wait_for(jobs, job_id, status):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        snapshot = jobs.get(job_id, result=True)
        if snapshot['status'] == status: return snapshot
        time.sleep(.005)
    pytest.fail(f'Planning job did not reach {status}: {snapshot}')


def test_progress_cancel_and_recovery(monkeypatch):
    entered = threading.Event()
    def plan(assembly, *, progress, cancelled):
        progress({'message': 'Checking parallel frame', 'paths_checked': 12})
        entered.set()
        while not cancelled(): time.sleep(.005)
        return {'status': 'cancelled', 'steps': []}
    monkeypatch.setattr('pipesim.planning.plan_build', plan)
    jobs = PlanningJobs()
    try:
        job = jobs.start(SimpleNamespace(parts={'a': 1}))
        assert entered.wait(5)
        assert jobs.get(job['id'])['progress']['paths_checked'] == 12
        assert jobs.get(job['id'], cancel=True)['cancel_requested']
        assert 'result' not in wait_for(jobs, job['id'], 'cancelled')
        monkeypatch.setattr('pipesim.planning.plan_build', lambda *a, **k: {'status': 'buildable', 'steps': [{'number': 1, 'kind': 'position'}]})
        next_job = jobs.start(SimpleNamespace(parts={}))
        result = wait_for(jobs, next_job['id'], 'completed')
        assert result['result']['status'] == 'buildable'
        assert 'result' not in jobs.get(next_job['id'])
        result['result']['steps'].clear()
        saved = jobs.get(next_job['id'], result=True)['result']
        assert saved['steps'] == [{'number': 1, 'kind': 'position'}]
        assert saved['instruction_steps'][0]['source_steps'] == [1]
    finally: jobs.close()


def test_new_search_supersedes_old_and_close_cancels(monkeypatch):
    def plan(assembly, *, progress, cancelled):
        while not cancelled(): time.sleep(.005)
        return {'status': 'cancelled', 'steps': []}
    monkeypatch.setattr('pipesim.planning.plan_build', plan)
    jobs = PlanningJobs()
    first = jobs.start(SimpleNamespace(parts={}))
    second = jobs.start(SimpleNamespace(parts={}))
    wait_for(jobs, first['id'], 'cancelled')
    jobs.close()
    wait_for(jobs, second['id'], 'cancelled')
    with pytest.raises(ValueError, match='shutting down'): jobs.start(SimpleNamespace(parts={}))


def test_failed_search_and_unknown_job(monkeypatch):
    def plan(*args, **kwargs): raise ValueError('invalid test geometry')
    monkeypatch.setattr('pipesim.planning.plan_build', plan)
    jobs = PlanningJobs()
    try:
        job = jobs.start(SimpleNamespace(parts={}))
        assert wait_for(jobs, job['id'], 'failed')['error'] == 'invalid test geometry'
        with pytest.raises(ValueError, match='Unknown'): jobs.get('missing')
    finally: jobs.close()

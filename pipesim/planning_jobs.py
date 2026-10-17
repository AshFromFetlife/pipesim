"""Background build planning with observable progress and cooperative cancellation."""
from collections import OrderedDict
import copy
import secrets
import threading
import time


class PlanningJobs:
    def __init__(self):
        self.lock = threading.Lock()
        self.jobs = OrderedDict()
        self.closed = False

    def start(self, assembly):
        from .planning import plan_build
        event = threading.Event()
        job = {'id': secrets.token_hex(16), 'status': 'running',
               'progress': {'message': 'Finding parallel sections and subassemblies',
                            'total_parts': len(assembly.parts)},
               'started': time.monotonic(), 'event': event}
        with self.lock:
            if self.closed:
                raise ValueError('The planning service is shutting down')
            # A new request supersedes the old design's search.
            for previous in self.jobs.values():
                if previous['status'] == 'running': previous['event'].set()
            self.jobs[job['id']] = job
            self._prune()

        def progress(value):
            with self.lock:
                job['progress'] = copy.deepcopy(value)

        def run():
            try:
                result = plan_build(assembly, progress=progress, cancelled=event.is_set)
                if result.get('status') == 'buildable' and not event.is_set():
                    from .build_presentation import instruction_steps
                    result['instruction_steps'] = instruction_steps(assembly, result)
                with self.lock:
                    if event.is_set() or result.get('status') == 'cancelled':
                        job['status'] = 'cancelled'
                    else:
                        job['result'] = result
                        job['status'] = 'completed'
            except Exception as exc:
                with self.lock:
                    job['status'] = 'cancelled' if event.is_set() else 'failed'
                    if not event.is_set(): job['error'] = str(exc)
            finally:
                with self.lock:
                    job['elapsed_s'] = time.monotonic() - job['started']
                    self._prune()

        threading.Thread(target=run, name='pipesim-build-plan', daemon=True).start()
        return self.get(job['id'])

    def _prune(self):
        completed = [key for key, value in self.jobs.items() if value['status'] != 'running']
        for key in completed[:-8]: self.jobs.pop(key)

    def get(self, job_id, *, cancel=False, result=False):
        with self.lock:
            job = self.jobs.get(job_id)
            if job is None: raise ValueError('Unknown build planning job')
            if cancel and job['status'] == 'running': job['event'].set()
            snapshot = {key: copy.deepcopy(job[key]) for key in ('id', 'status', 'progress', 'error') if key in job}
            snapshot['elapsed_s'] = job.get('elapsed_s', time.monotonic() - job['started'])
            snapshot['cancel_requested'] = job['event'].is_set()
            if result and 'result' in job: snapshot['result'] = copy.deepcopy(job['result'])
            return snapshot

    def close(self):
        with self.lock:
            self.closed = True
            for job in self.jobs.values(): job['event'].set()

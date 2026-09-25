"""Isolated editor workers: a native solver call cannot block Cancel or HTTP."""
import json
import multiprocessing
from pathlib import Path
import queue
import secrets
import tempfile
import threading
import time

from .document import DocumentError, plain


def _worker(assembly, options, directory, updates, log_progress):
    # The parent owns the whole temporary tree, even after a forced termination.
    tempfile.tempdir=directory
    from .physics import simulate
    from .simulation_control import console_progress
    def progress(value):
        try: updates.put_nowait(value)
        except queue.Full: pass
        if log_progress: console_progress(value)
    try:
        result=simulate(assembly,progress=progress,**options)
        payload={'result':plain(result)}
    except Exception as exc:
        payload={'error':str(exc)}
    target=Path(directory)/'result.json'
    pending=target.with_suffix('.tmp')
    pending.write_text(json.dumps(payload,allow_nan=False),encoding='utf-8')
    pending.replace(target)


class SimulationJobs:
    """One running simulation per editor; retain a bounded set of job results."""
    def __init__(self,*,log_progress=True):
        self.context=multiprocessing.get_context('spawn')
        self.log_progress=log_progress
        self.lock=threading.RLock()
        self.jobs={}

    def start(self,assembly,**options):
        from .physics import validate_simulation_options
        validate_simulation_options(**options)
        with self.lock:
            for job in self.jobs.values():
                self._refresh(job)
                if job['status']=='running':
                    raise DocumentError('A simulation is already running. Cancel it before starting another.')
            while len(self.jobs)>=4:
                self._dispose(self.jobs.pop(next(iter(self.jobs))))
            directory=tempfile.TemporaryDirectory(prefix='pipesim-job-')
            updates=self.context.Queue(maxsize=64)
            process=self.context.Process(target=_worker,args=(assembly,options,directory.name,updates,self.log_progress),daemon=True)
            job={'id':secrets.token_urlsafe(16),'status':'running','process':process,
                 'directory':directory,'updates':updates,'started':time.monotonic(),
                 'progress':{'phase':'starting','message':'Starting simulation worker','elapsed_s':0}}
            try: process.start()
            except BaseException:
                updates.close();directory.cleanup();raise
            self.jobs[job['id']]=job
            return self._public(job)

    def _refresh(self,job):
        if job['status']!='running': return
        while True:
            try: job['progress']=job['updates'].get_nowait()
            except queue.Empty: break
        process=job['process']
        target=Path(job['directory'].name)/'result.json'
        if target.exists():
            # Loading once avoids copying a large recording on every poll.
            payload=json.loads(target.read_text(encoding='utf-8'))
            job.update(payload)
            job['status']='failed' if 'error' in payload else 'completed'
            self._stop(job)
        elif not process.is_alive():
            job.update(status='failed',error=f"Simulation worker exited unexpectedly (code {process.exitcode})")
            self._stop(job)

    def _stop(self,job):
        process=job['process']
        if process.is_alive(): process.terminate()
        process.join(timeout=2)
        if process.is_alive(): process.kill();process.join(timeout=2)
        process.close()
        job['updates'].close()
        job['directory'].cleanup()

    def _dispose(self,job):
        if job['status']=='running': self._stop(job)

    def _public(self,job,include_result=False):
        fields=('id','status','progress','error')+ (('result',) if include_result else ())
        value={key:job[key] for key in fields if key in job}
        value['elapsed_s']=round(time.monotonic()-job['started'],2)
        return value

    def get(self,job_id,*,cancel=False,include_result=False):
        with self.lock:
            if job_id not in self.jobs: raise DocumentError('Simulation job not found; the server may have restarted')
            job=self.jobs[job_id]
            self._refresh(job)
            if cancel and job['status']=='running':
                self._stop(job)
                job['status']='cancelled'
                print('Simulation cancelled',flush=True)
            return self._public(job,include_result)

    def close(self):
        with self.lock:
            for job in self.jobs.values(): self._dispose(job)
            self.jobs.clear()


def simulate_in_worker(assembly,*,progress=None,**options):
    """Keep the CLI interruptible even while Bullet is inside a native call."""
    jobs=SimulationJobs(log_progress=False)
    try:
        job=jobs.start(assembly,**options);last=None
        while True:
            if progress and job['progress']!=last:
                progress(job['progress']);last=job['progress']
            if job['status']=='completed': return jobs.get(job['id'],include_result=True)['result']
            if job['status']=='failed': raise RuntimeError(job['error'])
            time.sleep(.1)
            job=jobs.get(job['id'])
    finally: jobs.close()

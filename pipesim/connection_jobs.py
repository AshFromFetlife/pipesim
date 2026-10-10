"""Cooperative cancellation for finite connection searches, including early cancel."""
import re
import threading
from collections import OrderedDict


class ConnectionCancelled(RuntimeError):
    pass


def check_cancelled(cancelled):
    if cancelled and cancelled():
        raise ConnectionCancelled('Connection search cancelled')


class ConnectionJobs:
    def __init__(self):
        self.lock=threading.Lock()
        self.running={}
        self.cancelled=OrderedDict()

    def _validate(self, job_id):
        if not isinstance(job_id,str) or not re.fullmatch(r'[A-Za-z0-9_-]{8,80}',job_id):
            raise ValueError('Invalid connection job id')

    def cancel(self, job_id):
        self._validate(job_id)
        with self.lock:
            if job_id in self.running: self.running[job_id].set()
            else:
                self.cancelled[job_id]=True
                while len(self.cancelled)>128: self.cancelled.popitem(last=False)
        return {'cancelled':True}

    def run(self, job_id, operation):
        if job_id is not None: self._validate(job_id)
        event=threading.Event()
        with self.lock:
            if job_id in self.running: raise ValueError('Connection job id already running')
            if self.cancelled.pop(job_id,None): return {'cancelled':True}
            key=job_id or object()
            self.running[key]=event
        try:
            check_cancelled(event.is_set)
            result=operation(event.is_set)
            check_cancelled(event.is_set)
            return result
        except ConnectionCancelled:
            return {'cancelled':True}
        finally:
            with self.lock: self.running.pop(key,None)

    def close(self):
        with self.lock:
            for event in self.running.values(): event.set()

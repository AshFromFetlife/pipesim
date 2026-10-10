import threading

from pipesim.connection_jobs import ConnectionJobs, check_cancelled


def test_connection_cancel_before_start_does_not_run_search():
    jobs=ConnectionJobs()
    jobs.cancel('connection-early')
    assert jobs.run('connection-early',lambda _:1/0)=={'cancelled':True}
    assert not jobs.running


def test_connection_cancel_stops_running_work_and_discards_late_result():
    jobs=ConnectionJobs();entered=threading.Event();release=threading.Event();results=[]
    def operation(cancelled):
        entered.set()
        assert release.wait(2)
        check_cancelled(cancelled)
        return {'document':'must not escape'}
    worker=threading.Thread(target=lambda:results.append(jobs.run('connection-running',operation)))
    worker.start()
    try:
        assert entered.wait(2)
        jobs.cancel('connection-running')
    finally:
        release.set();worker.join(3)
    assert results==[{'cancelled':True}]
    assert not jobs.running
    assert jobs.run('connection-next',lambda _: {'options':[]})=={'options':[]}


def test_completed_result_is_discarded_if_cancel_arrives_before_return():
    jobs=ConnectionJobs()
    def operation(cancelled):
        jobs.cancel('connection-last')
        return {'document':'must not escape'}
    assert jobs.run('connection-last',operation)=={'cancelled':True}

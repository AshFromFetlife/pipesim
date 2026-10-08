import pytest
import fuzz_runtime

from fuzz_runtime import batch_size, budget


def test_each_campaign_defaults_to_ten_minutes(monkeypatch):
    monkeypatch.delenv('PIPESIM_FUZZ_MINUTES', raising=False)
    minutes, started, deadline = budget()
    assert minutes == 10
    assert deadline - started == pytest.approx(600)
    assert batch_size(25, 100, 0, started, deadline, minutes) == 25


def test_short_budget_starts_with_one_case(monkeypatch):
    monkeypatch.setenv('PIPESIM_FUZZ_MINUTES', '0.02')
    minutes, started, deadline = budget()
    assert deadline - started == pytest.approx(1.2)
    assert batch_size(25, 100, 0, started, deadline, minutes) == 1


def test_later_batch_uses_observed_case_speed(monkeypatch):
    monkeypatch.setattr(fuzz_runtime.time, 'monotonic', lambda: 590)
    assert batch_size(25, 100, 100, 0, 600, 10) == 1


@pytest.mark.parametrize('value', ['0', '-1', 'nan', 'inf'])
def test_invalid_budget_is_rejected(monkeypatch, value):
    monkeypatch.setenv('PIPESIM_FUZZ_MINUTES', value)
    with pytest.raises(ValueError, match='positive finite'):
        budget()

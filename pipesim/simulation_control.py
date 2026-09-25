"""Progress and cancellation shared by Python, CLI and editor simulations."""
import time


class SimulationCancelled(Exception):
    """The caller cancelled; no partial recording should be published."""


class Progress:
    def __init__(self, callback=None, cancelled=None):
        self.callback = callback
        self.cancelled = cancelled
        self.started = time.monotonic()
        self.last = -float('inf')
        self.phase = None

    def check(self):
        if self.cancelled and self.cancelled():
            raise SimulationCancelled('Simulation cancelled')

    def update(self, phase, message, *, force=False, **values):
        self.check()
        now = time.monotonic()
        if force or phase != self.phase or now-self.last >= .5:
            self.last, self.phase = now, phase
            if self.callback:
                self.callback(dict(phase=phase, message=message,
                                   elapsed_s=round(now-self.started, 2), **values))


def console_progress(update, stream=None):
    """Flushed lines, also suitable for redirected stdout and notebook output."""
    text = f"[simulation {update['elapsed_s']:.1f}s] {update['message']}"
    if 'simulated_s' in update:
        text += f" | {update['simulated_s']:.3f}/{update['duration_s']:g}s simulated ({update['percent']:.1f}%)"
    if update.get('eta_s') is not None:
        text += f" | about {update['eta_s']:.0f}s remaining"
    print(text, file=stream, flush=True)

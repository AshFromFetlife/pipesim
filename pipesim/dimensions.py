"""Manufacturing/display precision, kept separate from authored geometry."""
from decimal import Decimal, ROUND_HALF_UP
import math
import re

DEFAULT_PRECISION_MM = 1.


def dimension_precision(assembly=None, override=None):
    value = (assembly.doc.get('build', {}).get('dimension_precision_mm', DEFAULT_PRECISION_MM)
             if override is None and assembly is not None else
             DEFAULT_PRECISION_MM if override is None else override)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not .001 <= value <= 10:
        raise ValueError('Dimension precision must be between 0.001 and 10 mm')
    return float(value)


def rounded_dimension(value, precision_mm=DEFAULT_PRECISION_MM):
    step = Decimal(str(dimension_precision(override=precision_mm)))
    return float((Decimal(str(value))/step).quantize(Decimal('1'), rounding=ROUND_HALF_UP)*step)


def cut_length(value, precision_mm=DEFAULT_PRECISION_MM):
    # A coarse display preference must never turn a real piece into a zero cut.
    return max(precision_mm, rounded_dimension(value, precision_mm))


def format_dimension(value, precision_mm=DEFAULT_PRECISION_MM):
    value = rounded_dimension(value, precision_mm)
    return format(Decimal(str(value)).normalize(), 'f') if value else '0'


def format_dimension_text(text, precision_mm=DEFAULT_PRECISION_MM):
    """Format dimension literals in instructions without changing IDs or angles."""
    def replacement(match):
        value=float(match[1])
        if text[:match.start()].lower().endswith('cut to '):
            value=cut_length(value,precision_mm)
        return format_dimension(value,precision_mm)
    return re.sub(r'(?<![\w.-])(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)(?= mm\b)',
                  replacement, text)

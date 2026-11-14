"""Persistent operator-selected telemetry limit; never a motion command."""
import json
import math

from .storage import ROOT, atomic_json

DEFAULT_OBSERVED_SPEED_MPS = .12
SDK_FLOAT_MAX = 3.4028234663852886e38
OBSERVED_SPEED_RANGE = (.01, SDK_FLOAT_MAX)
PARAMETER_RANGES = {
    'speed_mps': (.01, SDK_FLOAT_MAX), 'lateral_mps': (.01, SDK_FLOAT_MAX),
    'yaw_rps': (.02, SDK_FLOAT_MAX), 'linear_accel_mps2': (.01, SDK_FLOAT_MAX),
    'yaw_accel_rps2': (.01, SDK_FLOAT_MAX), 'model_duration_s': (.5, 86400.),
    'motion_duration_s': (.5, 86400.), 'startup_timeout_s': (1., 86400.),
    'max_travel_m': (.2, 10.),
    'observed_speed_limit_mps': OBSERVED_SPEED_RANGE,
}
OPTIONAL_DEFAULTS = dict(linear_accel_mps2=.30, yaw_accel_rps2=.60, startup_timeout_s=90., max_travel_m=2.)
LIMITS_PATH = ROOT/'config/execution_limits.json'


def observed_speed_limit(value):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or not OBSERVED_SPEED_RANGE[0] <= value <= OBSERVED_SPEED_RANGE[1]):
        raise ValueError('out_of_range:observed_speed_limit_mps')
    return float(value)


def validate_saved_limits(data):
    if not isinstance(data, dict) or set(data) != {'observed_speed_limit_mps'}:
        raise ValueError('invalid_execution_limits')
    return {'observed_speed_limit_mps': observed_speed_limit(data['observed_speed_limit_mps'])}


def load_saved_limits():
    if not LIMITS_PATH.exists():
        return {'observed_speed_limit_mps': DEFAULT_OBSERVED_SPEED_MPS}
    return validate_saved_limits(json.loads(LIMITS_PATH.read_text()))


def save_limits(data):
    saved = validate_saved_limits(data)
    atomic_json(LIMITS_PATH, saved)
    return saved

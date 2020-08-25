"""
Utility functions
"""
from typing import Dict, Any
import math
from geometry_msgs.msg import Quaternion


def init_from_dict(obj, values: Dict[str, Any]):
    """Assign every fields from a dict to object"""
    for name in values:
        value = values[name]
        if isinstance(value, dict):
            init_from_dict(obj.__getattribute__(name), value)
        else:
            obj.__setattr__(name, value)


def onehot_encode(value: int, max_v: int):
    """
    Encode an int value using onehot encoding
    :param value: The value to be encoded, must ranged in [0, max_v)
    :param max_v: The max possible value + 1
    """

    onehot = [0 for _ in range(max_v)]
    onehot[value] = 1
    return onehot


def init_quaternion(values: Quaternion):
    """Convert a quaternion to list of scalars"""
    q = [values.x, values.y, values.z, values.w]
    return q


def quart_to_rpy(x, y, z, w):
    """Convert a quaternion to rpy"""
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    p = math.asin(2 * (w * y - z * x))
    y = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return r, p, y

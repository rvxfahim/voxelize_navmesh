import ctypes
import numpy as np


def to_yup(xyz_zup: np.ndarray) -> np.ndarray:
    out = xyz_zup.copy()
    out[:, 1] = xyz_zup[:, 2]
    out[:, 2] = xyz_zup[:, 1]
    return out


def to_zup(xyz_yup: np.ndarray) -> np.ndarray:
    return to_yup(xyz_yup)


def vec3f(*args):
    if len(args) == 1:
        v = args[0]
        return (ctypes.c_float * 3)(float(v[0]), float(v[1]), float(v[2]))
    return (ctypes.c_float * 3)(float(args[0]), float(args[1]), float(args[2]))

from enum import Enum


class OpCodes(Enum):
    INVALID = -1
    STOP = 0
    SMOOTH = 2
    NORMAL = 4
    ORIENTATION_AVERAGE = 8
    SPIN_CW = 16
    SPIN_CCW = 32

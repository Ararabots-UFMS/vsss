from enum import Enum


class OpCodes(Enum):
    INVALID = -1
    STOP = 0
    NORMAL = 1
    IGNORE_DISTANCE = 2
    SPIN_CW = 3
    SPIN_CCW = 4
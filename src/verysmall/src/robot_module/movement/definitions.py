from enum import Enum


class OpCodes(Enum):
    INVALID = -1
    STOP = 0
    NORMAL = 1
    SPIN_CW = 2
    SPIN_CCW = 3
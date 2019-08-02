from enum import Enum


class OpCodes(Enum):
    STOP = 0
    NORMAL = 1
    SPIN_CW = 2
    SPIN_CCW = 3
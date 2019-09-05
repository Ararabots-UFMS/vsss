import math
from random import randint
from typing import Tuple

# Pixel converter
arena_w = 150 # cm
arena_h = 130 # cm
step = 3 # cm

ball = (randint(10, arena_w - 10), randint(10, arena_h - 10))
obstacle = (randint(10, arena_w - 10), randint(10, arena_h - 10))
factor = 4.5

def getArenaSize():
    return arena_w, arena_h

def getImageSize() -> Tuple[int, int]:

    img_w = arena_w * factor
    img_h = arena_h * factor

    return int(img_w), int(img_h)

def convertToPixel(cm: float) -> int:

    return round(cm * factor)

def convertToCm(px: float) -> int:
    
    return round(px / factor)
    
import math
import numpy as np


class Vec2:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def to_list(self) -> list:
        return [self.x, self.y]

    def to_np(self) -> np.ndarray:
        return np.array(self.to_list())

    def __add__(self, vec):
        try:
            return Vec2(self.x + vec.x, self.y + vec.y)
        except TypeError:
            return NotImplemented
        
    def __radd__(self, vec):
        return self + vec

    def __sub__(self, vec):
        _v = Vec2(self.x - vec.x, self.y - vec.y)
        return _v

    def __rsub__(self, vec):
        _v = Vec2(vec.x - self.x, vec.y - self.y)
        return _v

    def __eq__(self, vec):
        if isinstance(vec, Vec2):
            return self.x == vec.x and self.y == vec.y
        return False

    def __mul__(self, alpha):
        """ alpha is a scalar number """
        return Vec2(alpha*self.x, alpha*self.y)

    def __rmul__(self, alpha):
        """ alpha is a scalar number """
        return Vec2(alpha*self.x, alpha*self.y)

    def __truediv__(self, alpha):
        return Vec2(self.x/alpha, self.y/alpha)

    def __div__(self, alpha):
        return Vec2(self.x/alpha, self.y/alpha)

    def __repr__(self):
        return "Vec2(%r, %r)" % (self.x, self.y)

    def __abs__(self):
        return math.sqrt(self.x*self.x + self.y*self.y)

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def __getitem__(self, index):
        if index == 0:
            return self.x 
        elif index == 1:
            return self.y
        
        raise IndexError


class ObjState():
    def __init__(self, id:int = 0):
        self.id = id
        self.pos = Vec2()
        self.velocity = Vec2()
        self.orientation = 0
        # Size should be the largest axis of the object
        self.size = np.inf

    def set_pos(self, x, y) -> None:
        self.pos.x = x
        self.pos.y = y

    def set_speed(self, vx, vy) -> None:
        self.velocity.x = vx
        self.velocity.y = vy

    def set_orientation(self, theta) -> None:
        # For convention the orientation is always taken in relation
        # with the x axis
        self.orientation = theta

    def flat(self) -> list:
        flat_obj = [self.id] + self.pos.to_list() + self.velocity.to_list() + [self.orientation]
        return flat_obj

    def __repr__(self):
        return "ObjsectSate(id: %r pos: %r speed: %r theta: %r size: %r)" % (self.id, self.pos, self.velocity, self.orientation, self.size)

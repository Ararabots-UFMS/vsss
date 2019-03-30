import math


class Vec2:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def to_list(self):
        return [self.x, self.y]

    def to_np(self):
        return np.array(self.to_list())

    def __add__(self, vec):
        return Vec2(self.x + vec.x, self.y + vec.y)

    def __radd__(self, vec):
        _v = Vec2(self.x + vec.x, self.y + vec.y)
        return _v

    def __sub__(self, vec):
        _v = Vec2(self.x - vec.x, self.y - vec.y)
        return _v

    def __rsub__(self, vec):
        _v = Vec2(vec.x - self.x, vec.y - self.y)
        return _v

    def __mul__(self, alpha):
        """ alpha is a scalar number """
        return Vec2(alpha*self.x, alpha*self.y)

    def __rmul__(self, alpha):
        """ alpha is a scalar number """
        return Vec2(alpha*self.x, alpha*self.y)

    def __truediv__(self, alpha):
        return Vec2(self.x/alpha, self.y/alpha)

    # def __rdiv__(self, alpha):
    #     _v = Vec2(self.x/alpha, self.y/alpha)
    #     return _v
    # QUESTION: ISSO FAZ SENTIDO ?

    def __repr__(self):
        return "Vec2(%r, %r)" % (self.x, self.y)

    def __abs__(self):
        return math.sqrt(self.x*self.x + self.y*self.y)

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def __getitem__(self, index):
        if (index%2) == 0:
            return self.x
        return self.y

class ObjState():
    def __init__(self, id):
        self.id = id
        self.pos = Vec2()
        self.speed = Vec2()
        self.orientation = 0
        # Size should be the largest axis of the object
        self.size = math.inf

    def set_pos(self, x, y):
        self.pos.x = x
        self.pos.y = y

    def set_speed(self, vx, vy):
        self.speed.x = vx
        self.speed.y = vy

    def set_orientation(self, theta):
        # For convention the orientation is always taken in relation
        # with the x axis
        self.orientation = theta

    def flat(self):
        flat_obj = [self.id] + self.pos.to_list() + self.speed.to_list() + [self.orientation]
        return flat_obj

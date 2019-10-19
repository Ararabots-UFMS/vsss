from collections import deque
import numpy as np
import time

class MovingBody:
    def __init__(self, buffer_size=15):
        self._position = np.array([0, 0])

        self.position_buffer_x = deque((0 for _ in range(buffer_size)), maxlen=buffer_size)
        self.position_buffer_y = deque((0 for _ in range(buffer_size)), maxlen=buffer_size)

        self.speed_buffer_x = deque((0 for _ in range(buffer_size)), maxlen=buffer_size)
        self.speed_buffer_y = deque((0 for _ in range(buffer_size)), maxlen=buffer_size)

        t0 = time.time()
        self.time_buffer = deque((t0 for i in range(buffer_size)), maxlen=buffer_size)

        self._last_update = t0
        self._speed = np.array([.0, .0])
        self.orientation = .0

    def position_prediction(self, seconds=0.5):
        fitx = np.polyfit(self.time_buffer, self.position_buffer_x, 1)
        fity = np.polyfit(self.time_buffer, self.position_buffer_y, 1)
        px = np.poly1d(fitx)
        py = np.poly1d(fity)

        dt = time.time() + seconds
        p = px(dt), py(dt)
        return np.array(p)

    def get_time_on_axis(self, axis, value) -> float:
        if axis == 0:
            ds = value - self.position[0]
            dt = ds / self._speed[0]
        else:
            ds = value - self.position[1]
            dt = ds / self._speed[1]
        
        if dt < 0 or abs(dt) > 2: 
            dt = 0

        return dt
    
    @property
    def speed(self) -> np.ndarray:
        return self._speed
    
    @property
    def position(self) -> np.ndarray:
        return self._position
    
    @position.setter
    def position(self, position: np.ndarray) -> None:
        t = time.time()
        dt = t - self._last_update
        last_pos = self.position
        self._speed = 0.1 * (position - last_pos) / dt + 0.9 * self._speed

        self.position_buffer_x.append(position[0])
        self.position_buffer_y.append(position[1])
        self.time_buffer.append(float(time.time()))
        
        self._last_update = t
        self._position = position

    def __repr__(self):
        return "--position: " + str(self.position) + \
               "--speed: " + str(self.speed) + \
               "--orientation: " + str(self.orientation)

import numpy as np
from cv2 import inRange


class ColorTuple:

    def __init__(self, h=0, threshold=5):
        self.thresh = threshold
        self._low = np.array([self.over_clamp(h - self.thresh), 50, 50])
        self._high = np.array([self.over_clamp(h + self.thresh), 255, 255])

    def __str__(self):
        return "Low range is {} and high range is {} \n".format(str(self._low), str(self._high))

    def __call__(self):
        return [self._low, self._high]

    @staticmethod
    def over_clamp(value, min_val=0, max_val=179):
        if value < min_val:
            return max_val + (value - min_val)
        elif value > max_val:
            return min_val + (value - max_val)
        return value

    @staticmethod
    def mod_array_first(val_array, mod):
        new_array = val_array.copy()
        new_array[0] = ColorTuple.over_clamp(val_array[0] + mod)
        return new_array

    @property
    def low(self):
        return self._low

    @low.setter
    def low(self, value):
        if type(value) == np.ndarray:
            self._low = self.mod_array_first(value, -5)
        elif type(value) == int:
            self._low[0] = self.over_clamp(value - self.thresh)

    @property
    def high(self):
        return self._high

    @high.setter
    def high(self, value):
        if type(value) == np.ndarray:
            self._high = self.mod_array_first(value, 5)
        elif type(value) == int:
            self._high[0] = self.over_clamp(value + self.thresh)


class Ranges:

    def __init__(self):
        self.self_big = ColorTuple()
        self.self_small = ColorTuple()
        self.op_big = ColorTuple()
        self.op_small = ColorTuple()
        self.ball = ColorTuple()
        self.field = ColorTuple()
        self.mappings = {
            'q': self.self_big,
            'w': self.self_small,
            'a': self.op_big,
            's': self.op_small,
            'z': self.ball,
            'x': self.field
        }

    def update(self, key, hsv_val):
        if key not in self.mappings.keys():
            raise ValueError("Invalid key!")
        self.mappings[key].high = hsv_val
        self.mappings[key].low = hsv_val

    def masking(self, hsv_image):
        masks = {
            'self_big': inRange(hsv_image, *self.self_big()),
            'self_small': inRange(hsv_image, *self.self_small()),
            'op_big': inRange(hsv_image, *self.op_big()),
            'op_small': inRange(hsv_image, *self.op_small()),
            'ball': inRange(hsv_image, *self.ball()),
            'field': inRange(hsv_image, *self.field())
        }
        return masks

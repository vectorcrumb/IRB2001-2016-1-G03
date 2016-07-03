import numpy as np
from cv2 import inRange, cvtColor, COLOR_HSV2BGR
from collections import defaultdict


class ColorTuple:

    def __init__(self, h=0, threshold=5):
        self.thresh = threshold
        self._low = np.array([self.over_clamp(h - self.thresh), 120, 120])
        self._high = np.array([self.over_clamp(h + self.thresh), 255, 255])
        self._avr = np.hstack((self._low, self._low)).mean(axis=0)

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
    def mod_array_first(val_array, modh, mods, modv):
        new_array = val_array.copy()
        new_array[0] = ColorTuple.over_clamp(val_array[0] + modh)
        new_array[1] = ColorTuple.over_clamp(val_array[1] + mods, min_val=0, max_val=255)
        new_array[2] = ColorTuple.over_clamp(val_array[2] + modv, min_val=0, max_val=255)
        return new_array

    @property
    def avr(self):
        return self._avr

    @property
    def low(self):
        return self._low

    @low.setter
    def low(self, val):
        try:
            value, flag = val
        except ValueError:
            raise ValueError("Incorrectly set HSV values.")
        else:
            if type(value) == np.ndarray:
                # If flag == True, do not modify np array
                if flag:
                    self._low = value
                else:
                    self._low = self.mod_array_first(value, -5, -30, -30)
            elif type(value) == int:
                self._low[0] = self.over_clamp(value - self.thresh)
        self._avr = np.hstack((self._low, self._low)).mean(axis=0)

    @property
    def high(self):
        return self._high

    @high.setter
    def high(self, val):
        try:
            value, flag = val
        except ValueError:
            raise ValueError("Incorrectly set HSV values.")
        else:
            if type(value) == np.ndarray:
                # If flag == True, do not modify np array
                if flag:
                    self._high = value
                else:
                    self._high = self.mod_array_first(value, 5, 30, 30)
            elif type(value) == int:
                self._high[0] = self.over_clamp(value + self.thresh)
        self._avr = np.hstack((self._low, self._low)).mean(axis=0)


# noinspection PyArgumentList
class Ranges:

    def __init__(self, threshold=5):
        self.self_big = ColorTuple(threshold=threshold)
        self.self_small = ColorTuple(threshold=threshold)
        self.op_big = ColorTuple(threshold=threshold)
        self.op_small = ColorTuple(threshold=threshold)
        self.ball = ColorTuple(threshold=threshold)
        self.field = ColorTuple(threshold=threshold)
        self.mappings = {
            'q': self.self_big,
            'w': self.self_small,
            'a': self.op_big,
            's': self.op_small,
            'z': self.ball,
            'x': self.field
        }
        self.centroid_colors = None

    def update(self, key, hsv_val):
        if key not in self.mappings.keys():
            raise ValueError("Invalid key!")
        print("Modified", end=' ')
        if key == 'q':
            print("self_big")
        elif key == 'w':
            print("self_small")
        elif key == 'a':
            print("op_big")
        elif key == 's':
            print("op_small")
        elif key == 'z':
            print("ball")
        elif key == 'x':
            print("field")
        self.mappings[key].high = (hsv_val, False)
        self.mappings[key].low = (hsv_val, False)

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

    def def_centroid_colors(self):
        cen_colors = {
            'self_big': 255 - (cvtColor(self.self_big.avr, COLOR_HSV2BGR)),
            'self_small': 255 - (cvtColor(self.self_small.avr, COLOR_HSV2BGR)),
            'op_big': 255 - (cvtColor(self.op_big.avr, COLOR_HSV2BGR)),
            'op_small': 255 - (cvtColor(self.op_small.avr, COLOR_HSV2BGR)),
            'ball': 255 - (cvtColor(self.ball.avr, COLOR_HSV2BGR)),
            'field': 255 - (cvtColor(self.field.avr, COLOR_HSV2BGR))
        }
        self.centroid_colors = defaultdict(lambda: (255, 255, 255), cen_colors)

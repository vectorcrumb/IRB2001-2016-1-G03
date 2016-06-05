import numpy as np
import cv2

class FieldFinder:

    def __init__(self, color_data):
        if color_data is not dict:
            raise ValueError("color_data is not a dictionary")
        try:
            self.ball = [color_data['ball_lower'], color_data['ball_upper']]
            self.own_big = [color_data['own_big_lower'], color_data['own_big_upper']]
            self.own_small = [color_data['own_small_lower'], color_data['own_small_upper']]
            self.other_big = [color_data['other_big_lower'], color_data['other_big_upper']]
            self.other_small = [color_data['other_small_lower'], color_data['other_small_upper']]
            self.field = [color_data['field_lower'], color_data['field_upper']]
        except KeyError:
            raise ValueError("color_data is missing colors")

    def process_image(self, image):
        pass

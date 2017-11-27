#!/usr/bin/python

from scipy.ndimage.measurements import label
import cv2 as cv2
import numpy as np
import sys
import time

"""Lane Detector
Pass in [] into the constructor to receive a list of tuples for debugging purposes.
"""

class LaneDetector():
    def __init__(self):
        self.middle_of_car = (200 + 460)
        self.modified_width = 1272
        self.modified_height = 376

    def hsl_channel_threshold(hls, l_thresh=(180., 255), h_thresh=(0, 30)):
        l_channel = hls[:, :, 1]
        h_channel = hls[:, :, 0]

        # Threshold lightness channel and hue
        binary = np.zeros_like(l_channel)
        binary[((h_channel > h_thresh[0]) & (h_channel <= h_thresh[1]))
                 | ((l_channel > l_thresh[0]) & (l_channel <= l_thresh[1]))] = 1

        return binary

    # not used yet
    def colour_threshold(self, image, thresh=(210, 255)):
        combined = np.zeros_like(image[:, :, 0])
        combined[(image[:, :, 0] > thresh[0]) & (image[:, :, 1] > thresh[0]) & (image[:, :, 2] > thresh[0])
                 & (image[:, :, 0] < thresh[1]) & (image[:, :, 1] < thresh[1]) & (image[:, :, 2] < thresh[1])] = 255
        return combined


    def warp_image(self, image, mode='normal'):
        warp_vertices_src = [(75, image.shape[0]), (300, 185), (385, 185), (597, image.shape[0])]  # both sides
        src = np.float32(warp_vertices_src)
        dst = np.float32([[375, image.shape[0]], [375, 0], [697, 0], [697, image.shape[0]]])

        if mode == 'normal':
            M = cv2.getPerspectiveTransform(src, dst)
            warped = cv2.warpPerspective(image, M, (1272, image.shape[0]), flags=cv2.INTER_LINEAR)
        elif mode == 'inverse':
            M = cv2.getPerspectiveTransform(dst, src)
            warped = cv2.warpPerspective(image, M, (672, image.shape[0]), flags=cv2.INTER_LINEAR)
        return warped

    # returns lines, debug_image
    def find_all_lines(self, image):
        target_horizon = 150
        labeled_array, num_features = label(image)

        image_height = image.shape[0]
        y_axis = np.linspace(0, image_height - 1, num=image_height)


        debug_image = np.zeros([image.shape[0], image.shape[1], 3], dtype=np.uint8)
        debug_image[labeled_array > 0] = 255

        lines = []

        def get_line_fit(points, y_axis):
            y, x = np.nonzero(points)
            # ignore small blocks
            if ((np.max(y) - np.min(y) < 150) & (np.max(x) - np.min(x) < 150)):
                return None

            fit = np.polyfit(y, x, 2)
            line_fitx = fit[0] * y_axis ** 2 + fit[1] * y_axis + fit[2]
            # line_fitx = fit[0] * y_axis + fit[1]
            return line_fitx

        def get_closest_line(lines, target):
            all_lines = np.array(lines)
            all_lines_from_middle = all_lines[:, target_horizon] - self.middle_of_car

            closest_line = np.argmin(np.absolute(all_lines_from_middle), axis=0)
            return closest_line

        for feature in range(1, num_features + 1):
            feature_image = (labeled_array == feature)
            try:
                line_fit = get_line_fit(feature_image, y_axis)
                if line_fit is not None:
                    lines.append(line_fit)
                    points = np.vstack((line_fit, y_axis)).T.astype(np.int32)   ###Debug
                    cv2.polylines(debug_image, [points], False, (255, 0, 0), 3)   ###Debug
            except:
                print("Unexpected error:", sys.exc_info()[0])

        # find target line
        target_line = None
        if len(lines) > 0:
            left_lines = []
            right_lines = []
            for line in lines:  # total the lines to the left and right of the car separately
                if line[target_horizon] < self.middle_of_car:
                    left_lines.append(line)
                if line[target_horizon] >= self.middle_of_car:
                    right_lines.append(line)

            if len(left_lines) == 0:
                closest_right = get_closest_line(right_lines, self.middle_of_car)
                right = right_lines[closest_right]
                left = right - 600  # this is a guess of lane width
            elif len(right_lines) == 0:
                closest_left = get_closest_line(left_lines, self.middle_of_car)
                left = left_lines[closest_left]
                right = left + 600
            else:
                closest_left = get_closest_line(left_lines, self.middle_of_car)
                left = left_lines[closest_left]
                closest_right = get_closest_line(right_lines, self.middle_of_car)
                right = right_lines[closest_right]
            target_line = (left + right) / 2 # find middle of left and right
        else:
            target_line = np.full(image_height, self.middle_of_car)
        points = np.vstack((target_line, y_axis)).T.astype(np.int32)   ###Debug
        cv2.polylines(debug_image, [points], False, (0, 255, 0), 2)   ###Debug
        cv2.line(debug_image, (self.middle_of_car, 0), (self.middle_of_car, image_height), (0, 0, 255), 2)   ###Debug
        cv2.line(debug_image, (0, target_horizon), (self.modified_width, target_horizon), (0, 0, 255), 2)  ###Debug

        target_line = target_line - self.middle_of_car


        return target_line, cv2.flip(debug_image, 0)

    def process_images(self, left):
        # # convert to HLS
        left_hls = cv2.cvtColor(left, cv2.COLOR_BGR2HLS)

        # filter out lanes and warp it
        left_filtered = self.hsl_channel_threshold(left_hls)
        # left_filtered = self.colour_threshold(left)
        # left_filtered = cv2.copyMakeBorder(left_filtered, 185, 0, 0, 0, cv2.BORDER_CONSTANT, 0) #adding the top back


        left_warped = self.warp_image(left_filtered)

        left_flip = cv2.flip(left_warped, 0)

        target_line, debug_image = self.find_all_lines(left_flip)



        return target_line, debug_image


if __name__ == "__main__":
    #TODO
    pass

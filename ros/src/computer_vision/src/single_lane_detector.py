#!/usr/bin/python

from scipy.ndimage.measurements import label
import cv2 as cv2
import numpy as np
import sys
import time

"""Lane Detector
Pass in [] into the constructor to receive a list of tuples for debugging purposes.
"""

class SingleLaneDetector():
    def __init__(self):
        self.middle_of_car = (200 + 460)
        self.modified_width = 1072
        self.modified_height = 376

    def hsl_channel_threshold(self, hls, l_thresh=(0., 50)):
        l_channel = hls[:, :, 1]

        # Threshold lightness channel
        l_binary = np.zeros_like(l_channel)
        l_binary[(l_channel > l_thresh[0]) & (l_channel <= l_thresh[1])] = 1

        return l_binary

    # not used yet
    def colour_threshold(self, image, thresh=(210, 255)):
        combined = np.zeros_like(image[:, :, 0])
        combined[(image[:, :, 0] > thresh[0]) & (image[:, :, 1] > thresh[0]) & (image[:, :, 2] > thresh[0])
                 & (image[:, :, 0] < thresh[1]) & (image[:, :, 1] < thresh[1]) & (image[:, :, 2] < thresh[1])] = 255
        return combined

    def filter_lanes(self, image):
        hsl_channel_binary = self.hsl_channel_threshold(image, l_thresh=(210., 255.))

        combined = np.zeros_like(hsl_channel_binary)  # this makes it 1 channel
        combined[(hsl_channel_binary == 1)] = 1
        return combined

    def cut_image(self, image):
        return image[185:, :] #cut the top

    def warp_image(self, image, mode='normal'):
        warp_vertices_src = [(75, image.shape[0]), (300, 185), (385, 185), (597, image.shape[0])]  # both sides
        src = np.float32(warp_vertices_src)
        dst = np.float32([[375, image.shape[0]], [375, 0], [697, 0], [697, image.shape[0]]])

        if mode == 'normal':
            M = cv2.getPerspectiveTransform(src, dst)
            warped = cv2.warpPerspective(image, M, (1072, image.shape[0]), flags=cv2.INTER_LINEAR)
        elif mode == 'inverse':
            M = cv2.getPerspectiveTransform(dst, src)
            warped = cv2.warpPerspective(image, M, (672, image.shape[0]), flags=cv2.INTER_LINEAR)
        return warped

    # returns lines, debug_image
    def find_all_lines(self, image):
        labeled_array, num_features = label(image)

        image_height = image.shape[0]
        y_axis = np.linspace(0, image_height - 1, num=image_height)


        debug_image = np.zeros([image.shape[0], image.shape[1], 3], dtype=np.uint8)
        debug_image[labeled_array > 0] = 255

        lines = []

        def get_line_fit(points, y_axis):
            y, x = np.nonzero(points)
            # ignore small blocks
            if ((np.max(y) - np.min(y) < 100) & (np.max(x) - np.min(x) < 100)):
                return None

            fit = np.polyfit(y, x, 2)
            line_fitx = fit[0] * y_axis ** 2 + fit[1] * y_axis + fit[2]
            # line_fitx = fit[0] * y_axis + fit[1]
            return line_fitx

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
            all_lines = np.array(lines)
            all_lines_from_middle = all_lines[:, 0] - self.middle_of_car

            closest_line = np.argmin(np.absolute(all_lines_from_middle), axis=0)
            target_line = lines[closest_line]
        else:
            target_line = np.full(image_height, self.middle_of_car)

        # debug_image = cv2.copyMakeBorder(debug_image, 0, 50, 0, 0, cv2.BORDER_CONSTANT, 0)
        points = np.vstack((target_line, y_axis)).T.astype(np.int32)   ###Debug
        cv2.polylines(debug_image, [points], False, (0, 255, 0), 2)   ###Debug
        cv2.line(debug_image, (self.middle_of_car, 0), (self.middle_of_car, image_height), (0, 0, 255), 2)   ###Debug

        target_line = target_line - self.middle_of_car


        return target_line, cv2.flip(debug_image, 0)

    def process_images(self, left):
        # unused_top_height = 185
        # left_cut = left[185:, :] #cut the top
        #
        # # convert to HLS
        # left_image = cv2.cvtColor(left_cut, cv2.COLOR_BGR2HLS)


        left_hls = cv2.cvtColor(left, cv2.COLOR_BGR2HLS)

        # filter out lanes and warp it
        left_filtered = self.filter_lanes(left_hls)
        # left_filtered = self.colour_threshold(left)

        # left_filtered = cv2.copyMakeBorder(left_filtered, 185, 0, 0, 0, cv2.BORDER_CONSTANT, 0) #adding the top back


        left_warped = self.warp_image(left_filtered)

        left_flip = cv2.flip(left_warped, 0)

        target_line, debug_image = self.find_all_lines(left_flip)



        return target_line, debug_image


if __name__ == "__main__":
    #TODO
    pass

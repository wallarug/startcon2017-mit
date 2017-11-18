#!/usr/bin/python

from scipy.ndimage.measurements import label
import cv2 as cv2
import numpy as np
import time
import rospy

class LaneDetector():
    def __init__(self, debug=False):
        self.middle_of_car = (250 + 700) / 2
        self.modified_width = 872
        self.modified_height = 376
        self.debug = debug

    def abs_sobel_thresh(self, hls, orient='x', sobel_kernel=3, thresh=(0, 255)):

        # Convert to HLS and use lightness as the gray
        L = hls[:, :, 1]
        # 2) Take the derivative in x or y given orient = 'x' or 'y'
        if orient == 'x':
            sobel = cv2.Sobel(L, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
        else:
            sobel = cv2.Sobel(L, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
        # 3) Take the absolute value of the derivative or gradient
        abs_sobel = np.absolute(sobel)
        # 4) Scale to 8-bit (0 - 255) then convert to type = np.uint8
        scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel))
        # 5) Create a mask of 1's where the scaled gradient magnitude
        # is > thresh_min and < thresh_max
        sxbinary = np.zeros_like(scaled_sobel)
        sxbinary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1
        # 6) Return this mask as your binary_output image
        binary_output = sxbinary
        return binary_output

    def hsl_channel_threshold(self, hls, l_thresh=(0., 50)):
        l_channel = hls[:, :, 1]

        # Threshold lightness channel
        l_binary = np.zeros_like(l_channel)
        l_binary[(l_channel > l_thresh[0]) & (l_channel <= l_thresh[1])] = 1

        return l_binary

    def filter_lanes(self, image):
        #         x_grad_binary = self.abs_sobel_thresh(image, orient='x', thresh=(20, 60))
        #         y_grad_binary = self.abs_sobel_thresh(image, orient='y', thresh=(20, 60))
        hsl_channel_binary = self.hsl_channel_threshold(image, l_thresh=(0., 40.))

        combined = np.zeros_like(hsl_channel_binary)  # this makes it 1 channel
        #         combined[(hsl_channel_binary == 1) & (x_grad_binary == 1) & (y_grad_binary == 1)] = 1
        combined[(hsl_channel_binary == 1)] = 1

        return combined

    def warp_image(self, image, mode='normal', side="left"):
        #         image = np.copy(image)
        if side == "left":
            warp_vertices_src = [(75, image.shape[0]), (225, 250), (336, 250), (336, image.shape[0])]
            src = np.float32(warp_vertices_src)
            dst = np.float32([[275, image.shape[0]], [275, 0], [536, 0], [536, image.shape[0]]])
        else:
            warp_vertices_src = [(597, image.shape[0]), (447, 250), (336, 250), (336, image.shape[0])]
            src = np.float32(warp_vertices_src)
            dst = np.float32([[597, image.shape[0]], [597, 0], [336, 0], [336, image.shape[0]]])

        if mode == 'normal':
            M = cv2.getPerspectiveTransform(src, dst)
            warped = cv2.warpPerspective(image, M, (872, 376), flags=cv2.INTER_LINEAR)
        elif mode == 'inverse':
            M = cv2.getPerspectiveTransform(dst, src)
            warped = cv2.warpPerspective(image, M, (672, 376), flags=cv2.INTER_LINEAR)
        return warped

    def clean_image(self, image, side="left"):
        if side == "left":
            image[:, -300:] = 0
            image[-50:, -372:] = 0
        else:
            image[:, 0:300] = 0
            image[-50:, :450] = 0
        return image  # all the stuff below takes too long
        # labels = label(image)  # labels[0] = image with labels, labels[1] = count of labels
        # result = labels[0]
        #
        # for object_no in range(1, labels[1] + 1):
        #     # Find pixels with each label value
        #     nonzero = (result == object_no).nonzero()
        #     # Identify x and y values of those pixels
        #     nonzeroy = np.array(nonzero[0])
        #     nonzerox = np.array(nonzero[1])
        #     if ((np.max(nonzeroy) - np.min(nonzeroy) < 5) & (np.max(nonzerox) - np.min(nonzerox) < 5)):
        #         result[(result == object_no)] = 0
        # result[result > 0] = 1
        # return result

    def get_line_fit(self, points, y_axis):
        y, x = np.nonzero(points)
        fit = np.polyfit(y, x, 2)
        line_fitx = fit[0] * y_axis ** 2 + fit[1] * y_axis + fit[2]
        return line_fitx

    def process_images(self, left, right):
        # start = time.time()

        # convert to HLS
        left_image = cv2.cvtColor(left, cv2.COLOR_BGR2HLS).astype(np.float)
        right_image = cv2.cvtColor(right, cv2.COLOR_BGR2HLS).astype(np.float)

        # rospy.logwarn("time for HLS: %s", rospy.get_time() - start)
        # start = time.time()

        # filter out lanes and warp it
        left_filtered = self.filter_lanes(left_image)
        right_filtered = self.filter_lanes(right_image)

        # rospy.logwarn("time for filter: %s", rospy.get_time() - start)
        # start = time.time()
        # self.debug_left_filtered = left_filtered  ######debug
        # self.debug_left_warped = left_warped  ######debug
        left_warped = self.warp_image(left_filtered, side="left")
        right_warped = self.warp_image(right_filtered, side="right")
        # self.debug_right_filtered = right_filtered  ######debug
        # self.debug_right_warped = right_warped  ######debug
        # rospy.logwarn("time for warp: %s", rospy.get_time() - start)
        # start = time.time()

        # clean and flip the image upside down to make the front of the car x = 0
        left_clean = self.clean_image(left_warped, side="left")
        right_clean = self.clean_image(right_warped, side="right")

        # rospy.logwarn("time for cleaning: %s", rospy.get_time() - start)
        # start = time.time()

        # self.debug_left_clean = left_clean  ######debug
        # self.debug_right_clean = right_clean  ######debug
        left_clean = cv2.flip(left_clean, 0)
        right_clean = cv2.flip(right_clean, 0)

        # rospy.logwarn("time for flipping: %s", rospy.get_time() - start)
        # start = time.time()

        # fit a line to the left and right
        image_height = left_clean.shape[0]
        y_axis = np.linspace(-50, image_height - 1, num=image_height + 50)
        y, x = np.nonzero(left_clean)
        left_fitx = []
        if len(y) > 0:
            left_fitx = self.get_line_fit(left_clean, y_axis)
        y, x = np.nonzero(right_clean)
        right_fitx = []
        if len(y) > 0:
            right_fitx = self.get_line_fit(right_clean, y_axis)

        # rospy.logwarn("time for lines 1: %s", rospy.get_time() - start)
        # start = time.time()

        # find centre line
        print(image_height)
        if len(right_fitx) == 0:
            centre = left_fitx + self.middle_of_car / 2
        elif len(left_fitx) == 0:
            centre = right_fitx - self.middle_of_car / 2
        elif len(left_fitx) > 0 and len(right_fitx) > 0:
            centre = (left_fitx + right_fitx) / 2
        else:
            centre = np.full(len(y_axis), self.middle_of_car)

        # translate this to have the front of the car to be the positive x-axis
        self.x_axis = y_axis
        if len(left_fitx) > 0:
            self.left_y = (left_fitx - self.middle_of_car) * -1
        else:
            self.left_y = []
        if len(right_fitx) > 0:
            self.right_y = (right_fitx - self.middle_of_car) * -1
        else:
            self.right_y = []
        self.centre = centre - self.middle_of_car

        # rospy.logwarn("time for lines 2: %s", rospy.get_time() - start)
        # start = time.time()

        if self.debug == True:
            image = np.zeros([self.modified_height, self.modified_width, 3], dtype=np.uint8)
            y_axis = y_axis[51:][::-1]
            cv2.line(image, (self.middle_of_car, 0), (self.middle_of_car, self.modified_height), (255, 255, 255), 2)
            if len(left_fitx) > 0:
                points = np.vstack((left_fitx[51:], y_axis)).T.astype(np.int32)
                cv2.polylines(image, [points], False, (255, 0, 0), 2)
            if len(right_fitx) > 0:
                points = np.vstack((right_fitx[51:], y_axis)).T.astype(np.int32)
                cv2.polylines(image, [points], False, (0, 0, 255), 2)

            points = np.vstack((centre[51:], y_axis)).T.astype(np.int32)
            cv2.polylines(image, [points], False, (0, 255, 0), 2)
            self.debug_image = image

        # rospy.logwarn("time for the end: %s", rospy.get_time() - start)

    def get_path(self):
        return self.centre, self.x_axis

    def get_lanes(self):
        return self.left_y, self.right_y, self.x_axis

    def debug_lanes_as_image(self):
        return self.debug_image

    def debug_filtered(self):
        left = self.debug_left_filtered*255
        right = self.debug_right_filtered * 255
        return left.astype(np.uint8), right.astype(np.uint8)
        # return cv2.merge((left, left, left)).astype(np.uint8), cv2.merge((right, right, right)).astype(np.uint8)

    def debug_warped(self):
        left = self.debug_left_warped * 255
        right = self.debug_right_warped * 255
        return left.astype(np.uint8), right.astype(np.uint8)
        # return cv2.merge((left, left, left)).astype(np.uint8), cv2.merge((right, right, right)).astype(np.uint8)

    def debug_clean(self):
        left = self.debug_left_clean * 255
        right = self.debug_right_clean * 255
        return left.astype(np.uint8), right.astype(np.uint8)
        # return cv2.merge((left, left, left)).astype(np.uint8), cv2.merge((right, right, right)).astype(np.uint8)


if __name__ == "__main__":
    #TODO
    pass

#!/usr/bin/python

from scipy.ndimage.measurements import label
import cv2 as cv2
import numpy as np
import time

"""Lane Detector
Pass in [] into the constructor to receive a list of tuples for debugging purposes.
"""

class LaneDetector():
    def __init__(self, debugger=None):
        self.middle_of_car = (250 + 700) / 2
        self.modified_width = 872
        self.modified_height = 376
        self.debugger = debugger

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
        hsl_channel_binary = self.hsl_channel_threshold(image, l_thresh=(210., 255.))

        combined = np.zeros_like(hsl_channel_binary)  # this makes it 1 channel
        #         combined[(hsl_channel_binary == 1) & (x_grad_binary == 1) & (y_grad_binary == 1)] = 1
        combined[(hsl_channel_binary == 1)] = 1
        return combined

    def cut_image(self, image):
        return image[250:, :] #cut the top 250px

    def warp_image(self, image, mode='normal', side="left"):
        #         image = np.copy(image)
        if side == "left":
            warp_vertices_src = [(75, image.shape[0]), (300, 185), (385, 185), (597, image.shape[0])]  # both sides
            src = np.float32(warp_vertices_src)
            dst = np.float32([[375, image.shape[0]], [375, 0], [697, 0], [697, image.shape[0]]])
        else:
            warp_vertices_src = [(597, image.shape[0]), (447, 0), (336, 0), (336, image.shape[0])]
            src = np.float32(warp_vertices_src)
            dst = np.float32([[597, image.shape[0]], [597, 0], [336, 0], [336, image.shape[0]]])

        if mode == 'normal':
            M = cv2.getPerspectiveTransform(src, dst)
            warped = cv2.warpPerspective(image, M, (1072, image.shape[0]), flags=cv2.INTER_LINEAR)
        elif mode == 'inverse':
            M = cv2.getPerspectiveTransform(dst, src)
            warped = cv2.warpPerspective(image, M, (672, image.shape[0]), flags=cv2.INTER_LINEAR)
        return warped

    def clean_image(self, image, side="left"):
        # if side == "left":
        #     image[:, -300:] = 0
        #     image[-50:, -372:] = 0
        # else:
        #     image[:, 0:300] = 0
        #     image[-50:, :450] = 0
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

    def process_images(self, left):
        # start = time.time()
        # left_cut = self.cut_image(left)

        # convert to HLS
        left_image = cv2.cvtColor(left, cv2.COLOR_BGR2HLS).astype(np.float)

        # filter out lanes and warp it
        left_filtered = self.filter_lanes(left_image)

        left_warped = self.warp_image(left_filtered)



        # clean and flip the image upside down to make the front of the car x = 0
        # left_clean = self.clean_image(left_warped, side="left")
        left_clean = left_warped

        left_clean = cv2.flip(left_clean, 0)


        # fit a line to the left and right
        # image_height = left_clean.shape[0]
        # y_axis = np.linspace(-50, image_height - 1, num=image_height + 50)
        # y, x = np.nonzero(left_clean)
        # left_fitx = []
        # if len(y) > 0:
        #     left_fitx = self.get_line_fit(left_clean, y_axis)
        #
        # # rospy.logwarn("time for lines 1: %s", rospy.get_time() - start)
        # # start = time.time()
        #
        # # find centre line
        # if len(right_fitx) == 0:
        #     centre = left_fitx + self.middle_of_car / 2
        # elif len(left_fitx) == 0:
        #     centre = right_fitx - self.middle_of_car / 2
        # elif len(left_fitx) > 0 and len(right_fitx) > 0:
        #     centre = (left_fitx + right_fitx) / 2
        # else:
        #     centre = np.full(len(y_axis), self.middle_of_car)
        #
        # # translate this to have the front of the car to be the positive x-axis
        # self.x_axis = y_axis
        # if len(left_fitx) > 0:
        #     self.left_y = (left_fitx - self.middle_of_car) * -1
        # else:
        #     self.left_y = []
        # if len(right_fitx) > 0:
        #     self.right_y = (right_fitx - self.middle_of_car) * -1
        # else:
        #     self.right_y = []
        # self.centre = centre - self.middle_of_car

        # rospy.logwarn("time for lines 2: %s", rospy.get_time() - start)
        # start = time.time()

        if self.debugger is not None:
            # record images throughout
            self.debugger.append(("input", left))
            # self.debugger.append(("cut", [left_cut, right_cut]))
            # warped_raw = self.warp_image(np.copy(left), side="left")
            # warped_inverse_raw = self.warp_image(warped_raw, mode='inverse', side="left")
            # self.debugger.append(("left_warp_raw", [warped_raw, warped_inverse_raw]))
            # warped_raw = self.warp_image(np.copy(right), side="right")
            # warped_inverse_raw = self.warp_image(warped_raw, mode='inverse', side="right")
            # self.debugger.append(("right_warp_raw", [warped_raw, warped_inverse_raw]))

            self.debugger.append(("filtered", self.build_image_from_mask(left_filtered)))
            self.debugger.append(("warped", self.build_image_from_mask(left_warped)))
            self.debugger.append(("clean", self.build_image_from_mask(cv2.flip(left_clean, 0))))

            # image = np.zeros([image_height, self.modified_width, 3], dtype=np.uint8)
            # y_axis = y_axis[51:][::-1]
            # cv2.line(image, (self.middle_of_car, 0), (self.middle_of_car, image_height), (255, 255, 255), 2)
            # if len(left_fitx) > 0:
            #     points = np.vstack((left_fitx[51:], y_axis)).T.astype(np.int32)
            #     cv2.polylines(image, [points], False, (255, 0, 0), 2)
            # if len(right_fitx) > 0:
            #     points = np.vstack((right_fitx[51:], y_axis)).T.astype(np.int32)
            #     cv2.polylines(image, [points], False, (0, 0, 255), 2)

            # points = np.vstack((centre[51:], y_axis)).T.astype(np.int32)
            # cv2.polylines(image, [points], False, (0, 255, 0), 2)
            # self.debugger.append(("lane-lines", [image]))

        # rospy.logwarn("time for the end: %s", rospy.get_time() - start)

    def get_path(self):
        return self.centre, self.x_axis

    def get_lanes(self):
        # return self.left_y, self.right_y, self.x_axis
        pass

    def build_image_from_mask(self, mask):
        image = np.zeros([mask.shape[0], mask.shape[1], 3], dtype=np.uint8)
        image[mask > 0] = 255
        return image


if __name__ == "__main__":
    #TODO
    pass

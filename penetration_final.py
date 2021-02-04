#!/usr/bin/env python
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from PIL import Image as IMG
from pyzbar import pyzbar
from clover import srv
import numpy as np
import rospy
import math
import cv2
import os


# Here some functional code for color correction
def find_min(b, vmin_b, N=N, s1=s1):
    # global N, s1
    while (b[vmin_b + 1] <= N * s1 / 100):
        vmin_b = vmin_b + 1
    return vmin_b


def find_max(b, vmax_b, N=N, s2=s2):
    # global N, s2
    while b[vmax_b - 1] > (N - ((N / 100) * s2)):
        vmax_b = vmax_b - 1

    if vmax_b < 255 - 1:
        vmax_b = vmax_b + 1
    return vmax_b


def put_in_range(intensity, max_value, min_value):
    if intensity > max_value:
        intensity = max_value
    elif intensity < min_value:
        intensity = min_value

    return intensity


def norm_values(intensity, max_, min_):
    for i in range(3):
        intensity[i] = (intensity[i] - min_[i]) * 255 / (max_[i] - min_[i])

    return intensity


class DroneNTI(self):
    '''
    Main class for NTI Olympie
    '''
    self.bridge = CvBridge()
    self.mask = list()
    self.color_counter = [0 for i in range(4)]

    # Mask section

    # Green
    self.lower_range_green = np.array([36, 0, 0])
    self.upper_range_green = np.array([86, 255, 255])

    # Red
    self.mask_red1 = cv2.inRange(hsv, (0, 50, 20), (5, 255, 255))
    self.mask_red2 = cv2.inRange(hsv, (175, 50, 20), (180, 255, 255))

    # Blue
    self.lower_blue = np.array([100, 150, 0])
    self.upper_blue = np.array([140, 255, 255])

    # Yellow
    self.lower_range_yellow = np.array([20, 100, 100])
    self.upper_range_yellow = np.array([30, 255, 255])

    def get_crop_image(self):
        '''
        Get cropped image for better package detection
        :return: cropped iamge
        '''
        return self.cam_img[self.img_height - self.height_cutted:self.img_height + self.height_cutted,
               self.img_width - self.width_cutted:self.img_width + self.width_cutted]

    def __init__(self):
        rospy.init_node('final')

        # Init all ros services to get all needed information
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.land = rospy.ServiceProxy('land', Trigger)

        self.cam_img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')

        img_width = int(self.cam_img.shape[1])
        img_height = int(self.cam_img.shape[0])

        # standart 320*240
        self.height_cutted = 100 / 2
        self.width_cutted = 320 / 2

        self.img_width = img_width / 2
        self.img_height = img_height / 2

        self.cropped_image = self.get_crop_image()

    def wait_arrival(self, tolerance=0.1):
        '''
        Function for waiting while copter is going to point
        :param tolerance: tolerance ._.
        :return: nothing
        '''
        while not rospy.is_shutdown():
            telem = get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.2)

    def land_wait(self):
        '''
        Get info
        :return: nothing
        '''
        self.land()
        while get_telemetry().armed:
            rospy.sleep(0.2)

    def colour_detect(self, img):
        '''
        Color mask creation
        T0 = 0 #yellow - products
        T1 = 0 #green - clothes
        T2 = 0 #blue - fragile packaging
        T3 = 0 #red - correspondence
        :param img: input image
        :return: mask for all colors
        '''
        # global mask_red, mask_green, mask_yellow, mask_blue, mask
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        self.mask_yellow = cv2.inRange(hsv, self.lower_range_yellow, self.upper_range_yellow)
        self.mask_green = cv2.inRange(hsv, self.lower_range_green, self.upper_range_green)
        self.mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        self.mask_red = cv2.bitwise_or(self.mask_red1, self.mask_red2)

        self.mask = (self.mask_yellow, self.mask_green, self.mask_blue, self.mask_red)

        return self.mask

    def count_contours(self, img, color_counter_):
        '''

        :param img: Input image
        :param color_counter_: Counter of one of the color
        :return: new counter
        '''
        image, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(self.cam_img, contours, -1, (0, 0, 0), 2)
        # Some fucking debug output
        # print("number of contours is "+str(len(contours)))

        # One more fucking debug output
        # print("areas are:"+str(cv2.contourArea(contours[0]))+" "+str(cv2.contourArea(contours[1]))) ### min area = 400

        for i in range(len(contours)):
            if cv2.contourArea(contours[i]) > 400:
                color_counter_ += 1
                i += 1
        return color_counter_

    def count_goods(self):
        '''
        Update all colors
        :return:
        '''
        self.cam_img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')

        corrected_image = color_correction(self.get_crop_image())
        mask = colour_detect(corrected_image)

        for i in range(len(self.color_counter)):
            self.color_counter[i] = count_contours(mask[i], self.color_counter[i])

        self.cam_img = corrected_image.copy()

    def color_correction(self, image):
        '''
        Color correction. Something like equalize Histogram for all colors but much better
        :param image: input image
        :return: corrected image
        '''
        filterFactor = 1
        h, w, _ = image.shape
        w -= 1
        h -= 1
        N = w * h

        b, g, r = [0 for i in range(256)], [0 for i in range(256)], [0 for i in range(256)]

        for i in range(h):
            for j in range(w):
                intensity = image[i][j]

                b[intensity[0]] = b[intensity[0]] + 1
                g[intensity[1]] = g[intensity[1]] + 1
                r[intensity[2]] = r[intensity[2]] + 1

        for i in range(1, 256):
            b[i] = b[i] + filterFactor * b[i - 1]
            g[i] = g[i] + filterFactor * g[i - 1]
            r[i] = r[i] + filterFactor * r[i - 1]

        vmin_b = 0
        vmin_g = 0
        vmin_r = 0

        vmax_b = 255 - 1
        vmax_g = 255 - 1
        vmax_r = 255 - 1

        s1 = 3
        s2 = 3

        vmin_b = find_min(b, vmin_b)
        vmin_g = find_min(g, vmin_g)
        vmin_r = find_min(r, vmin_r)

        vmax_b = find_max(b, vmax_b)
        vmax_g = find_max(g, vmax_g)
        vmax_r = find_max(r, vmax_r)

        for i in range(h):
            for j in range(w):
                intensity = image[i][j]

                intensity[0] = put_in_range(intensity[0], vmax_b, vmin_b)
                intensity[1] = put_in_range(intensity[1], vmax_g, vmin_g)
                intensity[2] = put_in_range(intensity[2], vmax_r, vmin_r)

                image[i][j] = intensity

        for i in range(h):
            for j in range(w):
                intensity = image[i][j]

                intensity = norm_values(intensity, (vmax_b, vmax_g, vmax_r), (vmin_b, vmin_g, vmin_r))

                image[i][j] = intensity
        return image

    def get_info_from_packages(self):
        '''
        Count all packages
        :return:
        '''
        for i in range(0, 4):
            navigate(x=0.45 + 3.6 / 4 * i, y=4.5, z=1.0, speed=0.5, frame_id='aruco_map', auto_arm=True)
            self.wait_arrival()
            self.count_goods()
            print(self.color_counter)
            cv2.imwrite("result" + str(i) + ".png", cam_img)

        print("Balance: " + str(sum(self.color_counter)) + " cargo")
        for i in range(len(T)):
            print("Type " + str(i) + ": " + str(self.color_counter[i]) + " cargo")

    def run(self):
        '''
        Main function
        :return: nothing
        '''

        navigate(x=0.1, y=0.1, z=1.0, speed=0.5, frame_id='body', auto_arm=True)  # lift_up
        self.wait_arrival()
        navigate(x=0.1, y=0.1, z=1.8, speed=0.5, frame_id='aruco_map', auto_arm=True)
        self.wait_arrival()

        navigate(x=0.5, y=1.8, z=1.8, speed=0.5, frame_id='aruco_map', auto_arm=True)  # lift_up
        self.wait_arrival()
        navigate(x=0, y=1.8, z=1.0, speed=0.5, frame_id='aruco_map', auto_arm=True)  # lift_up
        self.wait_arrival()

        navigate(x=0, y=4.5, z=1.0, speed=0.5, frame_id='aruco_map', auto_arm=True)
        self.wait_arrival()

        self.get_info_from_packages()

        navigate(x=1.8, y=2.7, z=1.8, speed=0.5, frame_id='aruco_map', auto_arm=True)
        self.wait_arrival()

        navigate(x=0.0, y=0.0, z=1.8, speed=0.5, frame_id='aruco_map', auto_arm=True)
        self.wait_arrival()
        navigate(x=0.0, y=0.0, z=1.0, speed=0.5, frame_id='aruco_map', auto_arm=True)
        self.wait_arrival()
        navigate(x=0.0, y=0.0, z=-0.5, speed=0.5, frame_id='body', auto_arm=True)
        self.wait_arrival()

        self.land_wait()


if __name__ == '__main__':
    drone = DroneNTI()

    drone.run()

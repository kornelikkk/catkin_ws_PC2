#!/usr/bin/env python3

from time import sleep

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String


class SighDetector:
    def __init__(self) -> None:
        self.image_sub = None
        self.pub_detect = None
        self.rate = None
        self.bridge = None

    def r_init_node(self):
        rospy.init_node('sigh_detector')
        self.rate = rospy.Rate(0.5)

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/camera/image', Image, self.callback)
        self.pub_detect = rospy.Publisher('/sigh_result', String, queue_size=10)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)

        # (rows,cols,channels) = cv_image.shape
        # if cols > 60 and rows > 60 :
        #     cv2.circle(cv_image, (50,50), 10, 255)

        self.check_rule(cv_image)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

    def get_dominant_color(self, image, n_colors):
        pixels = np.float32(image).reshape((-1, 3))
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, .1)
        flags = cv2.KMEANS_RANDOM_CENTERS
        flags, labels, centroids = cv2.kmeans(
            pixels, n_colors, None, criteria, 10, flags)
        palette = np.uint8(centroids)
        return palette[np.argmax(np.unique(labels, return_counts=True)[::-1])]

    def check_rule(self, frame):
        #cameraCapture = cv2.VideoCapture(0)

        #bridge = CvBridge()
        #frame = bridge.imgmsg_to_cv2(image_sub, "bgr8")

        #cv2.namedWindow('camera')  # DELETE

        #success, frame = cameraCapture.read()
        #while success:
        #cv2.waitKey(1) # DELETE
        #success, frame = cameraCapture.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(gray, 37)
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 50, param1=120, param2=40)

        if not circles is None:
            circles = np.uint16(np.around(circles))
            max_r, max_i = 0, 0
            for i in range(len(circles[:, :, 2][0])):
                if circles[:, :, 2][0][i] > 50 and circles[:, :, 2][0][i] > max_r:
                    max_i = i
                    max_r = circles[:, :, 2][0][i]
            x, y, r = circles[:, :, :][0][max_i]
            if y > r and x > r:
                square = frame[y-r:y+r, x-r:x+r]

                dominant_color = self.get_dominant_color(square, 2)
                if dominant_color[2] > 100:
                    pass
                    # print('NO')
                    # self.pub_detect.publish("NO")
                elif dominant_color[0] > 70:
                    zone_0 = square[
                        square.shape[0]*2//8:square.shape[0]*4//8,
                        square.shape[1]*1//8:square.shape[1]*3//8
                    ]
                    zone_0_color = self.get_dominant_color(zone_0, 1)

                    zone_1 = square[
                        square.shape[0]*1//8:square.shape[1]*3//8,
                        square.shape[1]*3//8:square.shape[0]*5//8
                    ]
                    zone_1_color = self.get_dominant_color(zone_1, 1)

                    zone_2 = square[
                        square.shape[0]*2//8:square.shape[0]*4//8,
                        square.shape[1]*5//8:square.shape[1]*8//8
                    ]
                    zone_2_color = self.get_dominant_color(zone_2, 1)

                    if zone_1_color[2] < 60:
                        if sum(zone_0_color) > sum(zone_2_color):
                            print('left_turn')
                            self.pub_detect.publish("LEFT")
                        else:
                            print('right_turn')
                            self.pub_detect.publish("RIGHT")

                    else:
                        if sum(zone_1_color) > sum(zone_0_color) and sum(zone_1_color) > sum(zone_2_color):
                            print('forward_moving')
                            self.pub_detect.publish("FORWARD")
                        elif sum(zone_0_color) > sum(zone_2_color):
                            print('left_turn')
                            self.pub_detect.publish("LEFT")
                        else:
                            print('right_turn')
                            self.pub_detect.publish("RIGHT")
                else:
                    self.pub_detect.publish("NO")
                    print('no_detection')

            for i in circles[0, :]:
                cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)
        #cv2.imshow('camera', frame)  # DELETE

        #cv2.destroyAllWindows()  # DELETE
        #cameraCapture.release()

    def start(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

        cv2.destroyAllWindows()

def main():
    sigh_detector = SighDetector()
    sigh_detector.r_init_node()
    sleep(5)
    sigh_detector.start()


if __name__ == '__main__':
    main()

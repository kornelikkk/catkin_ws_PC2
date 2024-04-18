#! /usr/bin/env python3

import rospy
import os, sys
from aruco import CameraView
from collections import defaultdict
from time import sleep
from cv2 import aruco
from geometry_msgs.msg import Transform
import cv2

class ArucoProcessor:
    def __init__(self):
        self._last_img = None
        cv2.namedWindow("detected aruco", cv2.WINDOW_NORMAL)
        self._tf_pub = rospy.Publisher("/detected_aruco", Transform)

    def on_image_updated_cb(self, image, tf):
        print("image updated")
        print(tf)
        self._last_img = image

    def loop(self):
        if self._last_img is None: return
        cv2.imshow("detected aruco", self._last_img)
        key = cv2.waitKey(1)
        if key == ord('q'):
            exit(0)



if __name__ == "__main__":
    rospy.init_node("aruco_detector_node")
    image_topic = rospy.get_param("~image_topic", "/camera/image")
    camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/camera_info")
    markers_dict = defaultdict(lambda: 0.05) # default marker size is 5cm
    markers_dict[1] = 0.099 # marker 14 is the 10cm marker
    

    camera_view = CameraView(marker_sizes_dict=markers_dict, base_frame_id="base_footprint", aruco_dict=aruco.DICT_5X5_100)
    camera_view.subscribe(image_topic, camera_info_topic)

    processor = ArucoProcessor()

    camera_view.set_callback(processor.on_image_updated_cb)
    while not rospy.is_shutdown():
        processor.loop()
    
    # rospy.spin()
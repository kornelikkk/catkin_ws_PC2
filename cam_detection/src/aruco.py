import rospy
import os, sys
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import threading
import time
from collections import defaultdict


class CameraView:
    def __init__(self, aruco_dict=aruco.DICT_ARUCO_ORIGINAL,
                 marker_sizes_dict: defaultdict(float)=defaultdict(lambda: 0.05), 
                 base_frame_id="base"):
        self._marker_sizes_dict = marker_sizes_dict
        aruco_dict_obj = aruco.getPredefinedDictionary(aruco_dict)
        self._detector = aruco.ArucoDetector(aruco_dict_obj, aruco.DetectorParameters())
        self._bridge = CvBridge()
        self._image = None
        self._timestamp = None
        self._camera_info = None
        self._image_sub = None
        self._camera_info_sub = None
        self._callback = None
        self._base_frame_id = base_frame_id
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._thread = threading.Thread(target=self._run, daemon=True, name="ImageProcessingThread")
        self._thread.start()

    def subscribe(self, image_topic, camera_info_topic):
        self._image_sub and self._image_sub.unregister()
        self._camera_info_sub and self._camera_info_sub.unregister()
        self._image = None
        self._camera_info = None
        self._timestamp = None
        self._image_sub = rospy.Subscriber(image_topic, Image, self._image_callback)
        self._camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self._camera_info_callback)

    def _image_callback(self, msg):
        self._timestamp = msg.header.stamp
        self._image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def set_callback(self, callback):
        self._callback = callback

    def _camera_info_callback(self, msg):
        self._camera_info = msg

    def _call_callback(self, image, transforms):
        if not self._callback:
            return
        try:
            thread = threading.Thread(target=self._callback, args=(image, transforms), daemon=True)
            thread.start()
        except Exception as e:
            rospy.logerr(f"Error in callback: {e}")

    def _run(self):
        last_image = None
        repeat = 0
        rate = rospy.Rate(30)
        while True:
            if self._image is None:
                self._image = np.zeros((480, 640, 3), dtype=np.uint8)
            if self._camera_info is None:
                rospy.logwarn_throttle(0.5, "camera info not received yet, so publishing raw image")
                ret_image = self._image
                transforms = []
            else:
                timestamp = self._timestamp
                image = self._image
                ret_image, transforms = self._process_markers(image, self._camera_info, timestamp)
            if last_image is ret_image and repeat < 100:
                rate.sleep()
                repeat += 1
                continue
            self._call_callback(ret_image, transforms)
            last_image = ret_image
            repeat = 0
            rate.sleep()

    def _estimate_pose(self, length, width, corners, camera_info):
        mtx = np.array(camera_info.K).reshape((3, 3))
        dist = np.array(camera_info.D)
        marker_pos = np.array([[-length/2, width/2, 0],
                                 [length/2, width/2, 0],
                                 [length/2, -width/2, 0],
                                 [-length/2, -width/2, 0]], dtype=np.float32)
        _, rvec, tvec = cv2.solvePnP(marker_pos, np.array(corners, dtype="float32"), mtx, dist)
        return rvec, tvec
    
    def _draw_axis(self, image, rvec, tvec, camera_info, length=0.1):
        mtx = np.array(camera_info.K).reshape((3, 3))
        dist = np.array(camera_info.D)
        
        axis = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]]).reshape(-1,3)
        imgpts, jac = cv2.projectPoints(axis, rvec, tvec, mtx, dist)
        imgpts = imgpts.astype(np.int32)
        image = cv2.line(image, tuple(imgpts[0].ravel()), tuple(imgpts[1].ravel()), (255,0,0), 3)
        image = cv2.line(image, tuple(imgpts[0].ravel()), tuple(imgpts[2].ravel()), (0,255,0), 3)
        image = cv2.line(image, tuple(imgpts[0].ravel()), tuple(imgpts[3].ravel()), (0,0,255), 3)
        return image
    
    def _publish_transform_camera_buffer(self, rvec, tvec, timestamp, child_frame_id, parent_frame_id) -> TransformStamped:
        rotation_matrix = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 1]],
                            dtype=float)
        rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame_id
        t.child_frame_id = child_frame_id + "_from_camera"
        t.transform.translation.x = tvec[0]
        t.transform.translation.y = tvec[1]
        t.transform.translation.z = tvec[2]
        q = tf.transformations.quaternion_from_matrix(rotation_matrix)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self._tf_buffer.set_transform(t, "default_authority")
        return child_frame_id

    def _find_transform_to_base(self, target_frame_id, source_frame_id, timestamp) -> TransformStamped:
        try:
            transform = self._tf_buffer.lookup_transform(target_frame_id, source_frame_id + "_from_camera", rospy.Time.now() - rospy.Duration(1), rospy.Duration(0))
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"failed to find transform from {source_frame_id} to {target_frame_id}: {e}")
            return None
        else:
            transform.header.stamp = timestamp
            transform.child_frame_id = source_frame_id
            self._tf_broadcaster.sendTransform(transform)
            return transform

    def _process_markers(self, image, camera_info, timestamp):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejectedImgPoints = self._detector.detectMarkers(gray)
        if ids is None:
            return image, []
        ret_image = aruco.drawDetectedMarkers(image.copy(), corners, ids)
        transforms = []
        processed_ids = []
        for mid, corners in zip(ids, corners):
            if mid[0] in processed_ids:
                rospy.logwarn_throttle(1.0, f"marker {mid[0]} duplicated, skipping")
                continue
            if mid[0] not in self._marker_sizes_dict:
                rospy.logwarn_throttle(1.0, f"marker {mid[0]} not in marker size dict, using default size: {self._marker_sizes_dict[mid[0]]} m")
            length = self._marker_sizes_dict[mid[0]]
            # in new OpenCV version, some of the functions deprecated so we need to implement our own
            rvec, tvec = self._estimate_pose(length, length, corners, camera_info)
            frame_name = f"marker_{mid[0]}"
            transform_to_camera = self._publish_transform_camera_buffer(rvec, tvec, timestamp, frame_name, camera_info.header.frame_id)
            transform_to_base = self._find_transform_to_base(self._base_frame_id, frame_name, timestamp)
            if not transform_to_camera:
                continue
            transforms.append(transform_to_camera)
            ret_image = self._draw_axis(ret_image, rvec, tvec, camera_info, length / 2)
            processed_ids.append(mid[0])
        return ret_image, transforms
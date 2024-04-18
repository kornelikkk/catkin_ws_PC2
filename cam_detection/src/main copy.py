import rospy
from geometry_msgs.msg import Twist
from aruco import CameraView
from collections import defaultdict
from cv2 import aruco
import cv2

class ArucoProcessor:
    def __init__(self):
        self._last_img = None
        self._aruco_detected = False
        cv2.namedWindow("detected aruco", cv2.WINDOW_NORMAL)

    def on_image_updated_cb(self, image, tf):
        print(tf)
        if 'marker_20' in tf:
            print("урррррраааа")
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            pub.publish(move_cmd)
            self._aruco_detected = True
        self._last_img = image
        # Проверяем, есть ли маркер 20
        

    def loop(self):
        if self._last_img is None:
            return
        cv2.imshow("detected aruco", self._last_img)
        key = cv2.waitKey(1)
        if key == ord('q'):
            exit(0)

if __name__ == "__main__":
    rospy.init_node("aruco_detector_and_move_forward")  # Инициализация узла ROS
    
    # Параметры для aruco_detector_node
    image_topic = rospy.get_param("~image_topic", "/camera/image")
    camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/camera_info")
    markers_dict = defaultdict(lambda: 0.05)  # default marker size is 5cm
    markers_dict[1] = 0.099  # marker 14 is the 10cm marker

    # Инициализация и подписка aruco_detector_node
    camera_view = CameraView(marker_sizes_dict=markers_dict, base_frame_id="base_footprint", aruco_dict=aruco.DICT_5X5_100)
    processor = ArucoProcessor()
    camera_view.set_callback(processor.on_image_updated_cb)
    camera_view.subscribe(image_topic, camera_info_topic)

    # Инициализация узла ROS для движения вперед
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Создание издателя для темы /cmd_vel
    rate = rospy.Rate(10)  # Частота публикации сообщений (10 Гц)

    # Создание сообщения Twist для движения вперед
    move_cmd = Twist()
    move_cmd.linear.x = 0.2  # Устанавливаем желаемую линейную скорость вдоль оси x

    # Публикация сообщения и выполнение обработки изображения параллельно
    while not rospy.is_shutdown():
        pub.publish(move_cmd)  # Публикуем сообщение для движения вперед
        processor.loop()  # Выполняем обработку изображения
        rate.sleep()  # Ожидаем, чтобы соблюсти частоту публикации

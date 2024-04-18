;; Auto-generated. Do not edit!


(when (boundp 'turn_pkg::Direction)
  (if (not (find-package "TURN_PKG"))
    (make-package "TURN_PKG"))
  (shadow 'Direction (find-package "TURN_PKG")))
(unless (find-package "TURN_PKG::DIRECTION")
  (make-package "TURN_PKG::DIRECTION"))
(unless (find-package "TURN_PKG::DIRECTIONREQUEST")
  (make-package "TURN_PKG::DIRECTIONREQUEST"))
(unless (find-package "TURN_PKG::DIRECTIONRESPONSE")
  (make-package "TURN_PKG::DIRECTIONRESPONSE"))

(in-package "ROS")





(defclass turn_pkg::DirectionRequest
  :super ros::object
  :slots (_x _angle ))

(defmethod turn_pkg::DirectionRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _angle (float __angle))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass turn_pkg::DirectionResponse
  :super ros::object
  :slots ())

(defmethod turn_pkg::DirectionResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass turn_pkg::Direction
  :super ros::object
  :slots ())

(setf (get turn_pkg::Direction :md5sum-) "08327713f314dd581e41738ebb789392")
(setf (get turn_pkg::Direction :datatype-) "turn_pkg/Direction")
(setf (get turn_pkg::Direction :request) turn_pkg::DirectionRequest)
(setf (get turn_pkg::Direction :response) turn_pkg::DirectionResponse)

(defmethod turn_pkg::DirectionRequest
  (:response () (instance turn_pkg::DirectionResponse :init)))

(setf (get turn_pkg::DirectionRequest :md5sum-) "08327713f314dd581e41738ebb789392")
(setf (get turn_pkg::DirectionRequest :datatype-) "turn_pkg/DirectionRequest")
(setf (get turn_pkg::DirectionRequest :definition-)
      "float32 x
float32 angle
---


")

(setf (get turn_pkg::DirectionResponse :md5sum-) "08327713f314dd581e41738ebb789392")
(setf (get turn_pkg::DirectionResponse :datatype-) "turn_pkg/DirectionResponse")
(setf (get turn_pkg::DirectionResponse :definition-)
      "float32 x
float32 angle
---


")



(provide :turn_pkg/Direction "08327713f314dd581e41738ebb789392")



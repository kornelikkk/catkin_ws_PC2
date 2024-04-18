; Auto-generated. Do not edit!


(cl:in-package turn_pkg-srv)


;//! \htmlinclude Direction-request.msg.html

(cl:defclass <Direction-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass Direction-request (<Direction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Direction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Direction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name turn_pkg-srv:<Direction-request> is deprecated: use turn_pkg-srv:Direction-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Direction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turn_pkg-srv:x-val is deprecated.  Use turn_pkg-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <Direction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turn_pkg-srv:angle-val is deprecated.  Use turn_pkg-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Direction-request>) ostream)
  "Serializes a message object of type '<Direction-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Direction-request>) istream)
  "Deserializes a message object of type '<Direction-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Direction-request>)))
  "Returns string type for a service object of type '<Direction-request>"
  "turn_pkg/DirectionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Direction-request)))
  "Returns string type for a service object of type 'Direction-request"
  "turn_pkg/DirectionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Direction-request>)))
  "Returns md5sum for a message object of type '<Direction-request>"
  "08327713f314dd581e41738ebb789392")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Direction-request)))
  "Returns md5sum for a message object of type 'Direction-request"
  "08327713f314dd581e41738ebb789392")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Direction-request>)))
  "Returns full string definition for message of type '<Direction-request>"
  (cl:format cl:nil "float32 x~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Direction-request)))
  "Returns full string definition for message of type 'Direction-request"
  (cl:format cl:nil "float32 x~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Direction-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Direction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Direction-request
    (cl:cons ':x (x msg))
    (cl:cons ':angle (angle msg))
))
;//! \htmlinclude Direction-response.msg.html

(cl:defclass <Direction-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Direction-response (<Direction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Direction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Direction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name turn_pkg-srv:<Direction-response> is deprecated: use turn_pkg-srv:Direction-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Direction-response>) ostream)
  "Serializes a message object of type '<Direction-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Direction-response>) istream)
  "Deserializes a message object of type '<Direction-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Direction-response>)))
  "Returns string type for a service object of type '<Direction-response>"
  "turn_pkg/DirectionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Direction-response)))
  "Returns string type for a service object of type 'Direction-response"
  "turn_pkg/DirectionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Direction-response>)))
  "Returns md5sum for a message object of type '<Direction-response>"
  "08327713f314dd581e41738ebb789392")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Direction-response)))
  "Returns md5sum for a message object of type 'Direction-response"
  "08327713f314dd581e41738ebb789392")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Direction-response>)))
  "Returns full string definition for message of type '<Direction-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Direction-response)))
  "Returns full string definition for message of type 'Direction-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Direction-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Direction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Direction-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Direction)))
  'Direction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Direction)))
  'Direction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Direction)))
  "Returns string type for a service object of type '<Direction>"
  "turn_pkg/Direction")
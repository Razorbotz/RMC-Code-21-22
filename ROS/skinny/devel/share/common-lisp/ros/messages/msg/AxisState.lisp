; Auto-generated. Do not edit!


(cl:in-package messages-msg)


;//! \htmlinclude AxisState.msg.html

(cl:defclass <AxisState> (roslisp-msg-protocol:ros-message)
  ((joystick
    :reader joystick
    :initarg :joystick
    :type cl:fixnum
    :initform 0)
   (axis
    :reader axis
    :initarg :axis
    :type cl:fixnum
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:float
    :initform 0.0))
)

(cl:defclass AxisState (<AxisState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AxisState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AxisState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name messages-msg:<AxisState> is deprecated: use messages-msg:AxisState instead.")))

(cl:ensure-generic-function 'joystick-val :lambda-list '(m))
(cl:defmethod joystick-val ((m <AxisState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:joystick-val is deprecated.  Use messages-msg:joystick instead.")
  (joystick m))

(cl:ensure-generic-function 'axis-val :lambda-list '(m))
(cl:defmethod axis-val ((m <AxisState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:axis-val is deprecated.  Use messages-msg:axis instead.")
  (axis m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <AxisState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:state-val is deprecated.  Use messages-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AxisState>) ostream)
  "Serializes a message object of type '<AxisState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joystick)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'axis)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AxisState>) istream)
  "Deserializes a message object of type '<AxisState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joystick)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'axis)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'state) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AxisState>)))
  "Returns string type for a message object of type '<AxisState>"
  "messages/AxisState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AxisState)))
  "Returns string type for a message object of type 'AxisState"
  "messages/AxisState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AxisState>)))
  "Returns md5sum for a message object of type '<AxisState>"
  "66b25cf96600974c62d570e9935b51ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AxisState)))
  "Returns md5sum for a message object of type 'AxisState"
  "66b25cf96600974c62d570e9935b51ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AxisState>)))
  "Returns full string definition for message of type '<AxisState>"
  (cl:format cl:nil "uint8 joystick~%uint8 axis~%float32 state~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AxisState)))
  "Returns full string definition for message of type 'AxisState"
  (cl:format cl:nil "uint8 joystick~%uint8 axis~%float32 state~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AxisState>))
  (cl:+ 0
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AxisState>))
  "Converts a ROS message object to a list"
  (cl:list 'AxisState
    (cl:cons ':joystick (joystick msg))
    (cl:cons ':axis (axis msg))
    (cl:cons ':state (state msg))
))

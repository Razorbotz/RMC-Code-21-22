; Auto-generated. Do not edit!


(cl:in-package messages-msg)


;//! \htmlinclude HatState.msg.html

(cl:defclass <HatState> (roslisp-msg-protocol:ros-message)
  ((joystick
    :reader joystick
    :initarg :joystick
    :type cl:fixnum
    :initform 0)
   (hat
    :reader hat
    :initarg :hat
    :type cl:fixnum
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass HatState (<HatState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HatState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HatState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name messages-msg:<HatState> is deprecated: use messages-msg:HatState instead.")))

(cl:ensure-generic-function 'joystick-val :lambda-list '(m))
(cl:defmethod joystick-val ((m <HatState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:joystick-val is deprecated.  Use messages-msg:joystick instead.")
  (joystick m))

(cl:ensure-generic-function 'hat-val :lambda-list '(m))
(cl:defmethod hat-val ((m <HatState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:hat-val is deprecated.  Use messages-msg:hat instead.")
  (hat m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <HatState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:state-val is deprecated.  Use messages-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HatState>) ostream)
  "Serializes a message object of type '<HatState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joystick)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hat)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HatState>) istream)
  "Deserializes a message object of type '<HatState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joystick)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hat)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HatState>)))
  "Returns string type for a message object of type '<HatState>"
  "messages/HatState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HatState)))
  "Returns string type for a message object of type 'HatState"
  "messages/HatState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HatState>)))
  "Returns md5sum for a message object of type '<HatState>"
  "b0f14c98250145be7590c9d7a97bb21b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HatState)))
  "Returns md5sum for a message object of type 'HatState"
  "b0f14c98250145be7590c9d7a97bb21b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HatState>)))
  "Returns full string definition for message of type '<HatState>"
  (cl:format cl:nil "uint8 joystick~%uint8 hat~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HatState)))
  "Returns full string definition for message of type 'HatState"
  (cl:format cl:nil "uint8 joystick~%uint8 hat~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HatState>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HatState>))
  "Converts a ROS message object to a list"
  (cl:list 'HatState
    (cl:cons ':joystick (joystick msg))
    (cl:cons ':hat (hat msg))
    (cl:cons ':state (state msg))
))

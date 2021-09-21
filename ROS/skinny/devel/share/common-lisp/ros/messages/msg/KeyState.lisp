; Auto-generated. Do not edit!


(cl:in-package messages-msg)


;//! \htmlinclude KeyState.msg.html

(cl:defclass <KeyState> (roslisp-msg-protocol:ros-message)
  ((key
    :reader key
    :initarg :key
    :type cl:fixnum
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass KeyState (<KeyState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name messages-msg:<KeyState> is deprecated: use messages-msg:KeyState instead.")))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <KeyState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:key-val is deprecated.  Use messages-msg:key instead.")
  (key m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <KeyState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:state-val is deprecated.  Use messages-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyState>) ostream)
  "Serializes a message object of type '<KeyState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'key)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'key)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyState>) istream)
  "Deserializes a message object of type '<KeyState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'key)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'key)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyState>)))
  "Returns string type for a message object of type '<KeyState>"
  "messages/KeyState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyState)))
  "Returns string type for a message object of type 'KeyState"
  "messages/KeyState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyState>)))
  "Returns md5sum for a message object of type '<KeyState>"
  "5b60023c4a21dcdf8c0309517484805f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyState)))
  "Returns md5sum for a message object of type 'KeyState"
  "5b60023c4a21dcdf8c0309517484805f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyState>)))
  "Returns full string definition for message of type '<KeyState>"
  (cl:format cl:nil "uint16 key~%uint8 state~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyState)))
  "Returns full string definition for message of type 'KeyState"
  (cl:format cl:nil "uint16 key~%uint8 state~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyState>))
  (cl:+ 0
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyState>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyState
    (cl:cons ':key (key msg))
    (cl:cons ':state (state msg))
))

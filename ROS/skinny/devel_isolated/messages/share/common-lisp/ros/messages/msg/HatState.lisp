; Auto-generated. Do not edit!


(cl:in-package messages-msg)


;//! \htmlinclude HatState.msg.html

(cl:defclass <HatState> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:integer
    :initform 0))
)

(cl:defclass HatState (<HatState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HatState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HatState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name messages-msg:<HatState> is deprecated: use messages-msg:HatState instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <HatState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:state-val is deprecated.  Use messages-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HatState>) ostream)
  "Serializes a message object of type '<HatState>"
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HatState>) istream)
  "Deserializes a message object of type '<HatState>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
  "7a2f37ef2ba405f0c7a15cc72663d6f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HatState)))
  "Returns md5sum for a message object of type 'HatState"
  "7a2f37ef2ba405f0c7a15cc72663d6f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HatState>)))
  "Returns full string definition for message of type '<HatState>"
  (cl:format cl:nil "int32 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HatState)))
  "Returns full string definition for message of type 'HatState"
  (cl:format cl:nil "int32 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HatState>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HatState>))
  "Converts a ROS message object to a list"
  (cl:list 'HatState
    (cl:cons ':state (state msg))
))

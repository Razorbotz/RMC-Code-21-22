; Auto-generated. Do not edit!


(cl:in-package communication-msg)


;//! \htmlinclude JoystickCommand.msg.html

(cl:defclass <JoystickCommand> (roslisp-msg-protocol:ros-message)
  ((axisX
    :reader axisX
    :initarg :axisX
    :type cl:float
    :initform 0.0)
   (axisY
    :reader axisY
    :initarg :axisY
    :type cl:float
    :initform 0.0))
)

(cl:defclass JoystickCommand (<JoystickCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JoystickCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JoystickCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name communication-msg:<JoystickCommand> is deprecated: use communication-msg:JoystickCommand instead.")))

(cl:ensure-generic-function 'axisX-val :lambda-list '(m))
(cl:defmethod axisX-val ((m <JoystickCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader communication-msg:axisX-val is deprecated.  Use communication-msg:axisX instead.")
  (axisX m))

(cl:ensure-generic-function 'axisY-val :lambda-list '(m))
(cl:defmethod axisY-val ((m <JoystickCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader communication-msg:axisY-val is deprecated.  Use communication-msg:axisY instead.")
  (axisY m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JoystickCommand>) ostream)
  "Serializes a message object of type '<JoystickCommand>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'axisX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'axisY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JoystickCommand>) istream)
  "Deserializes a message object of type '<JoystickCommand>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'axisX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'axisY) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JoystickCommand>)))
  "Returns string type for a message object of type '<JoystickCommand>"
  "communication/JoystickCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JoystickCommand)))
  "Returns string type for a message object of type 'JoystickCommand"
  "communication/JoystickCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JoystickCommand>)))
  "Returns md5sum for a message object of type '<JoystickCommand>"
  "2ef0f7484f7f056d7fcb586ad36e7d96")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JoystickCommand)))
  "Returns md5sum for a message object of type 'JoystickCommand"
  "2ef0f7484f7f056d7fcb586ad36e7d96")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JoystickCommand>)))
  "Returns full string definition for message of type '<JoystickCommand>"
  (cl:format cl:nil "float32 axisX~%float32 axisY~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JoystickCommand)))
  "Returns full string definition for message of type 'JoystickCommand"
  (cl:format cl:nil "float32 axisX~%float32 axisY~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JoystickCommand>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JoystickCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'JoystickCommand
    (cl:cons ':axisX (axisX msg))
    (cl:cons ':axisY (axisY msg))
))

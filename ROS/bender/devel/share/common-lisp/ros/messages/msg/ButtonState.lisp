; Auto-generated. Do not edit!


(cl:in-package messages-msg)


;//! \htmlinclude ButtonState.msg.html

(cl:defclass <ButtonState> (roslisp-msg-protocol:ros-message)
  ((number
    :reader number
    :initarg :number
    :type cl:integer
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:integer
    :initform 0))
)

(cl:defclass ButtonState (<ButtonState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ButtonState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ButtonState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name messages-msg:<ButtonState> is deprecated: use messages-msg:ButtonState instead.")))

(cl:ensure-generic-function 'number-val :lambda-list '(m))
(cl:defmethod number-val ((m <ButtonState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:number-val is deprecated.  Use messages-msg:number instead.")
  (number m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <ButtonState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:state-val is deprecated.  Use messages-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ButtonState>) ostream)
  "Serializes a message object of type '<ButtonState>"
  (cl:let* ((signed (cl:slot-value msg 'number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ButtonState>) istream)
  "Deserializes a message object of type '<ButtonState>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'number) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ButtonState>)))
  "Returns string type for a message object of type '<ButtonState>"
  "messages/ButtonState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ButtonState)))
  "Returns string type for a message object of type 'ButtonState"
  "messages/ButtonState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ButtonState>)))
  "Returns md5sum for a message object of type '<ButtonState>"
  "09160d212a568b0a18a64adb7eb7b9d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ButtonState)))
  "Returns md5sum for a message object of type 'ButtonState"
  "09160d212a568b0a18a64adb7eb7b9d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ButtonState>)))
  "Returns full string definition for message of type '<ButtonState>"
  (cl:format cl:nil "int32 number ~%int32 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ButtonState)))
  "Returns full string definition for message of type 'ButtonState"
  (cl:format cl:nil "int32 number ~%int32 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ButtonState>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ButtonState>))
  "Converts a ROS message object to a list"
  (cl:list 'ButtonState
    (cl:cons ':number (number msg))
    (cl:cons ':state (state msg))
))

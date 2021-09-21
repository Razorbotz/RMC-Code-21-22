; Auto-generated. Do not edit!


(cl:in-package messages-msg)


;//! \htmlinclude TalonOut.msg.html

(cl:defclass <TalonOut> (roslisp-msg-protocol:ros-message)
  ((current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0)
   (voltage
    :reader voltage
    :initarg :voltage
    :type cl:float
    :initform 0.0)
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:float
    :initform 0.0)
   (position
    :reader position
    :initarg :position
    :type cl:integer
    :initform 0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:integer
    :initform 0))
)

(cl:defclass TalonOut (<TalonOut>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TalonOut>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TalonOut)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name messages-msg:<TalonOut> is deprecated: use messages-msg:TalonOut instead.")))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current-val is deprecated.  Use messages-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:voltage-val is deprecated.  Use messages-msg:voltage instead.")
  (voltage m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:temperature-val is deprecated.  Use messages-msg:temperature instead.")
  (temperature m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:position-val is deprecated.  Use messages-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:velocity-val is deprecated.  Use messages-msg:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TalonOut>) ostream)
  "Serializes a message object of type '<TalonOut>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'position)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'velocity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TalonOut>) istream)
  "Deserializes a message object of type '<TalonOut>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voltage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'position) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TalonOut>)))
  "Returns string type for a message object of type '<TalonOut>"
  "messages/TalonOut")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TalonOut)))
  "Returns string type for a message object of type 'TalonOut"
  "messages/TalonOut")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TalonOut>)))
  "Returns md5sum for a message object of type '<TalonOut>"
  "1dfab55edab53accf343c309dd48ae24")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TalonOut)))
  "Returns md5sum for a message object of type 'TalonOut"
  "1dfab55edab53accf343c309dd48ae24")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TalonOut>)))
  "Returns full string definition for message of type '<TalonOut>"
  (cl:format cl:nil "float32 current~%float32 voltage~%float32 temperature~%int32 position~%int32 velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TalonOut)))
  "Returns full string definition for message of type 'TalonOut"
  (cl:format cl:nil "float32 current~%float32 voltage~%float32 temperature~%int32 position~%int32 velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TalonOut>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TalonOut>))
  "Converts a ROS message object to a list"
  (cl:list 'TalonOut
    (cl:cons ':current (current msg))
    (cl:cons ':voltage (voltage msg))
    (cl:cons ':temperature (temperature msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
))

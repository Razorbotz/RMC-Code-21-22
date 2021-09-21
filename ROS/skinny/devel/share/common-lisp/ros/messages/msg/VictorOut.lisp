; Auto-generated. Do not edit!


(cl:in-package messages-msg)


;//! \htmlinclude VictorOut.msg.html

(cl:defclass <VictorOut> (roslisp-msg-protocol:ros-message)
  ((deviceID
    :reader deviceID
    :initarg :deviceID
    :type cl:integer
    :initform 0)
   (busVoltage
    :reader busVoltage
    :initarg :busVoltage
    :type cl:float
    :initform 0.0)
   (outputVoltage
    :reader outputVoltage
    :initarg :outputVoltage
    :type cl:float
    :initform 0.0)
   (outputPercent
    :reader outputPercent
    :initarg :outputPercent
    :type cl:float
    :initform 0.0))
)

(cl:defclass VictorOut (<VictorOut>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VictorOut>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VictorOut)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name messages-msg:<VictorOut> is deprecated: use messages-msg:VictorOut instead.")))

(cl:ensure-generic-function 'deviceID-val :lambda-list '(m))
(cl:defmethod deviceID-val ((m <VictorOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:deviceID-val is deprecated.  Use messages-msg:deviceID instead.")
  (deviceID m))

(cl:ensure-generic-function 'busVoltage-val :lambda-list '(m))
(cl:defmethod busVoltage-val ((m <VictorOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:busVoltage-val is deprecated.  Use messages-msg:busVoltage instead.")
  (busVoltage m))

(cl:ensure-generic-function 'outputVoltage-val :lambda-list '(m))
(cl:defmethod outputVoltage-val ((m <VictorOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:outputVoltage-val is deprecated.  Use messages-msg:outputVoltage instead.")
  (outputVoltage m))

(cl:ensure-generic-function 'outputPercent-val :lambda-list '(m))
(cl:defmethod outputPercent-val ((m <VictorOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:outputPercent-val is deprecated.  Use messages-msg:outputPercent instead.")
  (outputPercent m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VictorOut>) ostream)
  "Serializes a message object of type '<VictorOut>"
  (cl:let* ((signed (cl:slot-value msg 'deviceID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'busVoltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'outputVoltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'outputPercent))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VictorOut>) istream)
  "Deserializes a message object of type '<VictorOut>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'deviceID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'busVoltage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'outputVoltage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'outputPercent) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VictorOut>)))
  "Returns string type for a message object of type '<VictorOut>"
  "messages/VictorOut")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VictorOut)))
  "Returns string type for a message object of type 'VictorOut"
  "messages/VictorOut")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VictorOut>)))
  "Returns md5sum for a message object of type '<VictorOut>"
  "1308269e2bd460ac3f55ed9f4bcd8d51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VictorOut)))
  "Returns md5sum for a message object of type 'VictorOut"
  "1308269e2bd460ac3f55ed9f4bcd8d51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VictorOut>)))
  "Returns full string definition for message of type '<VictorOut>"
  (cl:format cl:nil "int32 deviceID~%float32 busVoltage~%float32 outputVoltage~%float32 outputPercent~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VictorOut)))
  "Returns full string definition for message of type 'VictorOut"
  (cl:format cl:nil "int32 deviceID~%float32 busVoltage~%float32 outputVoltage~%float32 outputPercent~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VictorOut>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VictorOut>))
  "Converts a ROS message object to a list"
  (cl:list 'VictorOut
    (cl:cons ':deviceID (deviceID msg))
    (cl:cons ':busVoltage (busVoltage msg))
    (cl:cons ':outputVoltage (outputVoltage msg))
    (cl:cons ':outputPercent (outputPercent msg))
))

; Auto-generated. Do not edit!


(cl:in-package messages-msg)


;//! \htmlinclude TalonOut.msg.html

(cl:defclass <TalonOut> (roslisp-msg-protocol:ros-message)
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
   (outputCurrent
    :reader outputCurrent
    :initarg :outputCurrent
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
    :initform 0.0)
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:float
    :initform 0.0)
   (sensorPosition
    :reader sensorPosition
    :initarg :sensorPosition
    :type cl:integer
    :initform 0)
   (sensorVelocity
    :reader sensorVelocity
    :initarg :sensorVelocity
    :type cl:integer
    :initform 0)
   (closedLoopError
    :reader closedLoopError
    :initarg :closedLoopError
    :type cl:integer
    :initform 0)
   (integralAccumulator
    :reader integralAccumulator
    :initarg :integralAccumulator
    :type cl:integer
    :initform 0)
   (errorDerivative
    :reader errorDerivative
    :initarg :errorDerivative
    :type cl:integer
    :initform 0))
)

(cl:defclass TalonOut (<TalonOut>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TalonOut>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TalonOut)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name messages-msg:<TalonOut> is deprecated: use messages-msg:TalonOut instead.")))

(cl:ensure-generic-function 'deviceID-val :lambda-list '(m))
(cl:defmethod deviceID-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:deviceID-val is deprecated.  Use messages-msg:deviceID instead.")
  (deviceID m))

(cl:ensure-generic-function 'busVoltage-val :lambda-list '(m))
(cl:defmethod busVoltage-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:busVoltage-val is deprecated.  Use messages-msg:busVoltage instead.")
  (busVoltage m))

(cl:ensure-generic-function 'outputCurrent-val :lambda-list '(m))
(cl:defmethod outputCurrent-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:outputCurrent-val is deprecated.  Use messages-msg:outputCurrent instead.")
  (outputCurrent m))

(cl:ensure-generic-function 'outputVoltage-val :lambda-list '(m))
(cl:defmethod outputVoltage-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:outputVoltage-val is deprecated.  Use messages-msg:outputVoltage instead.")
  (outputVoltage m))

(cl:ensure-generic-function 'outputPercent-val :lambda-list '(m))
(cl:defmethod outputPercent-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:outputPercent-val is deprecated.  Use messages-msg:outputPercent instead.")
  (outputPercent m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:temperature-val is deprecated.  Use messages-msg:temperature instead.")
  (temperature m))

(cl:ensure-generic-function 'sensorPosition-val :lambda-list '(m))
(cl:defmethod sensorPosition-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:sensorPosition-val is deprecated.  Use messages-msg:sensorPosition instead.")
  (sensorPosition m))

(cl:ensure-generic-function 'sensorVelocity-val :lambda-list '(m))
(cl:defmethod sensorVelocity-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:sensorVelocity-val is deprecated.  Use messages-msg:sensorVelocity instead.")
  (sensorVelocity m))

(cl:ensure-generic-function 'closedLoopError-val :lambda-list '(m))
(cl:defmethod closedLoopError-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:closedLoopError-val is deprecated.  Use messages-msg:closedLoopError instead.")
  (closedLoopError m))

(cl:ensure-generic-function 'integralAccumulator-val :lambda-list '(m))
(cl:defmethod integralAccumulator-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:integralAccumulator-val is deprecated.  Use messages-msg:integralAccumulator instead.")
  (integralAccumulator m))

(cl:ensure-generic-function 'errorDerivative-val :lambda-list '(m))
(cl:defmethod errorDerivative-val ((m <TalonOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:errorDerivative-val is deprecated.  Use messages-msg:errorDerivative instead.")
  (errorDerivative m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TalonOut>) ostream)
  "Serializes a message object of type '<TalonOut>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'outputCurrent))))
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'sensorPosition)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'sensorVelocity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'closedLoopError)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'integralAccumulator)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'errorDerivative)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TalonOut>) istream)
  "Deserializes a message object of type '<TalonOut>"
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
    (cl:setf (cl:slot-value msg 'outputCurrent) (roslisp-utils:decode-single-float-bits bits)))
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
      (cl:setf (cl:slot-value msg 'sensorPosition) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sensorVelocity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'closedLoopError) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'integralAccumulator) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'errorDerivative) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
  "d8f6fc5634dde6263b80ec7fc1aa9dee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TalonOut)))
  "Returns md5sum for a message object of type 'TalonOut"
  "d8f6fc5634dde6263b80ec7fc1aa9dee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TalonOut>)))
  "Returns full string definition for message of type '<TalonOut>"
  (cl:format cl:nil "int32 deviceID~%float32 busVoltage~%float32 outputCurrent~%float32 outputVoltage~%float32 outputPercent~%float32 temperature~%int32 sensorPosition~%int32 sensorVelocity~%int32 closedLoopError ~%int32 integralAccumulator~%int32 errorDerivative~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TalonOut)))
  "Returns full string definition for message of type 'TalonOut"
  (cl:format cl:nil "int32 deviceID~%float32 busVoltage~%float32 outputCurrent~%float32 outputVoltage~%float32 outputPercent~%float32 temperature~%int32 sensorPosition~%int32 sensorVelocity~%int32 closedLoopError ~%int32 integralAccumulator~%int32 errorDerivative~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TalonOut>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TalonOut>))
  "Converts a ROS message object to a list"
  (cl:list 'TalonOut
    (cl:cons ':deviceID (deviceID msg))
    (cl:cons ':busVoltage (busVoltage msg))
    (cl:cons ':outputCurrent (outputCurrent msg))
    (cl:cons ':outputVoltage (outputVoltage msg))
    (cl:cons ':outputPercent (outputPercent msg))
    (cl:cons ':temperature (temperature msg))
    (cl:cons ':sensorPosition (sensorPosition msg))
    (cl:cons ':sensorVelocity (sensorVelocity msg))
    (cl:cons ':closedLoopError (closedLoopError msg))
    (cl:cons ':integralAccumulator (integralAccumulator msg))
    (cl:cons ':errorDerivative (errorDerivative msg))
))

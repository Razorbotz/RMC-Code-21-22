; Auto-generated. Do not edit!


(cl:in-package messages-msg)


;//! \htmlinclude Power.msg.html

(cl:defclass <Power> (roslisp-msg-protocol:ros-message)
  ((voltage
    :reader voltage
    :initarg :voltage
    :type cl:float
    :initform 0.0)
   (current0
    :reader current0
    :initarg :current0
    :type cl:float
    :initform 0.0)
   (current1
    :reader current1
    :initarg :current1
    :type cl:float
    :initform 0.0)
   (current2
    :reader current2
    :initarg :current2
    :type cl:float
    :initform 0.0)
   (current3
    :reader current3
    :initarg :current3
    :type cl:float
    :initform 0.0)
   (current4
    :reader current4
    :initarg :current4
    :type cl:float
    :initform 0.0)
   (current5
    :reader current5
    :initarg :current5
    :type cl:float
    :initform 0.0)
   (current6
    :reader current6
    :initarg :current6
    :type cl:float
    :initform 0.0)
   (current7
    :reader current7
    :initarg :current7
    :type cl:float
    :initform 0.0)
   (current8
    :reader current8
    :initarg :current8
    :type cl:float
    :initform 0.0)
   (current9
    :reader current9
    :initarg :current9
    :type cl:float
    :initform 0.0)
   (current10
    :reader current10
    :initarg :current10
    :type cl:float
    :initform 0.0)
   (current11
    :reader current11
    :initarg :current11
    :type cl:float
    :initform 0.0)
   (current12
    :reader current12
    :initarg :current12
    :type cl:float
    :initform 0.0)
   (current13
    :reader current13
    :initarg :current13
    :type cl:float
    :initform 0.0)
   (current14
    :reader current14
    :initarg :current14
    :type cl:float
    :initform 0.0)
   (current15
    :reader current15
    :initarg :current15
    :type cl:float
    :initform 0.0))
)

(cl:defclass Power (<Power>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Power>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Power)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name messages-msg:<Power> is deprecated: use messages-msg:Power instead.")))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:voltage-val is deprecated.  Use messages-msg:voltage instead.")
  (voltage m))

(cl:ensure-generic-function 'current0-val :lambda-list '(m))
(cl:defmethod current0-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current0-val is deprecated.  Use messages-msg:current0 instead.")
  (current0 m))

(cl:ensure-generic-function 'current1-val :lambda-list '(m))
(cl:defmethod current1-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current1-val is deprecated.  Use messages-msg:current1 instead.")
  (current1 m))

(cl:ensure-generic-function 'current2-val :lambda-list '(m))
(cl:defmethod current2-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current2-val is deprecated.  Use messages-msg:current2 instead.")
  (current2 m))

(cl:ensure-generic-function 'current3-val :lambda-list '(m))
(cl:defmethod current3-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current3-val is deprecated.  Use messages-msg:current3 instead.")
  (current3 m))

(cl:ensure-generic-function 'current4-val :lambda-list '(m))
(cl:defmethod current4-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current4-val is deprecated.  Use messages-msg:current4 instead.")
  (current4 m))

(cl:ensure-generic-function 'current5-val :lambda-list '(m))
(cl:defmethod current5-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current5-val is deprecated.  Use messages-msg:current5 instead.")
  (current5 m))

(cl:ensure-generic-function 'current6-val :lambda-list '(m))
(cl:defmethod current6-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current6-val is deprecated.  Use messages-msg:current6 instead.")
  (current6 m))

(cl:ensure-generic-function 'current7-val :lambda-list '(m))
(cl:defmethod current7-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current7-val is deprecated.  Use messages-msg:current7 instead.")
  (current7 m))

(cl:ensure-generic-function 'current8-val :lambda-list '(m))
(cl:defmethod current8-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current8-val is deprecated.  Use messages-msg:current8 instead.")
  (current8 m))

(cl:ensure-generic-function 'current9-val :lambda-list '(m))
(cl:defmethod current9-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current9-val is deprecated.  Use messages-msg:current9 instead.")
  (current9 m))

(cl:ensure-generic-function 'current10-val :lambda-list '(m))
(cl:defmethod current10-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current10-val is deprecated.  Use messages-msg:current10 instead.")
  (current10 m))

(cl:ensure-generic-function 'current11-val :lambda-list '(m))
(cl:defmethod current11-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current11-val is deprecated.  Use messages-msg:current11 instead.")
  (current11 m))

(cl:ensure-generic-function 'current12-val :lambda-list '(m))
(cl:defmethod current12-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current12-val is deprecated.  Use messages-msg:current12 instead.")
  (current12 m))

(cl:ensure-generic-function 'current13-val :lambda-list '(m))
(cl:defmethod current13-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current13-val is deprecated.  Use messages-msg:current13 instead.")
  (current13 m))

(cl:ensure-generic-function 'current14-val :lambda-list '(m))
(cl:defmethod current14-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current14-val is deprecated.  Use messages-msg:current14 instead.")
  (current14 m))

(cl:ensure-generic-function 'current15-val :lambda-list '(m))
(cl:defmethod current15-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:current15-val is deprecated.  Use messages-msg:current15 instead.")
  (current15 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Power>) ostream)
  "Serializes a message object of type '<Power>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current6))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current7))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current8))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current9))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current10))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current11))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current12))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current13))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current14))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current15))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Power>) istream)
  "Deserializes a message object of type '<Power>"
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
    (cl:setf (cl:slot-value msg 'current0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current4) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current5) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current6) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current7) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current8) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current9) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current10) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current11) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current12) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current13) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current14) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current15) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Power>)))
  "Returns string type for a message object of type '<Power>"
  "messages/Power")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Power)))
  "Returns string type for a message object of type 'Power"
  "messages/Power")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Power>)))
  "Returns md5sum for a message object of type '<Power>"
  "a5bf87df7460476e356f8a0cdbf06096")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Power)))
  "Returns md5sum for a message object of type 'Power"
  "a5bf87df7460476e356f8a0cdbf06096")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Power>)))
  "Returns full string definition for message of type '<Power>"
  (cl:format cl:nil "float32 voltage~%float32 current0~%float32 current1~%float32 current2~%float32 current3~%float32 current4~%float32 current5~%float32 current6~%float32 current7~%float32 current8~%float32 current9~%float32 current10~%float32 current11~%float32 current12~%float32 current13~%float32 current14 ~%float32 current15~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Power)))
  "Returns full string definition for message of type 'Power"
  (cl:format cl:nil "float32 voltage~%float32 current0~%float32 current1~%float32 current2~%float32 current3~%float32 current4~%float32 current5~%float32 current6~%float32 current7~%float32 current8~%float32 current9~%float32 current10~%float32 current11~%float32 current12~%float32 current13~%float32 current14 ~%float32 current15~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Power>))
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
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Power>))
  "Converts a ROS message object to a list"
  (cl:list 'Power
    (cl:cons ':voltage (voltage msg))
    (cl:cons ':current0 (current0 msg))
    (cl:cons ':current1 (current1 msg))
    (cl:cons ':current2 (current2 msg))
    (cl:cons ':current3 (current3 msg))
    (cl:cons ':current4 (current4 msg))
    (cl:cons ':current5 (current5 msg))
    (cl:cons ':current6 (current6 msg))
    (cl:cons ':current7 (current7 msg))
    (cl:cons ':current8 (current8 msg))
    (cl:cons ':current9 (current9 msg))
    (cl:cons ':current10 (current10 msg))
    (cl:cons ':current11 (current11 msg))
    (cl:cons ':current12 (current12 msg))
    (cl:cons ':current13 (current13 msg))
    (cl:cons ':current14 (current14 msg))
    (cl:cons ':current15 (current15 msg))
))

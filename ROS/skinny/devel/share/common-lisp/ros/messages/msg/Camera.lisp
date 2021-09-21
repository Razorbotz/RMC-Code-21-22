; Auto-generated. Do not edit!


(cl:in-package messages-msg)


;//! \htmlinclude Camera.msg.html

(cl:defclass <Camera> (roslisp-msg-protocol:ros-message)
  ((port
    :reader port
    :initarg :port
    :type cl:integer
    :initform 0)
   (address
    :reader address
    :initarg :address
    :type cl:string
    :initform ""))
)

(cl:defclass Camera (<Camera>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Camera>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Camera)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name messages-msg:<Camera> is deprecated: use messages-msg:Camera instead.")))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <Camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:port-val is deprecated.  Use messages-msg:port instead.")
  (port m))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <Camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:address-val is deprecated.  Use messages-msg:address instead.")
  (address m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Camera>) ostream)
  "Serializes a message object of type '<Camera>"
  (cl:let* ((signed (cl:slot-value msg 'port)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'address))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'address))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Camera>) istream)
  "Deserializes a message object of type '<Camera>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'port) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'address) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'address) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Camera>)))
  "Returns string type for a message object of type '<Camera>"
  "messages/Camera")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Camera)))
  "Returns string type for a message object of type 'Camera"
  "messages/Camera")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Camera>)))
  "Returns md5sum for a message object of type '<Camera>"
  "5962e9b7032210cb9e9f696366c62cf3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Camera)))
  "Returns md5sum for a message object of type 'Camera"
  "5962e9b7032210cb9e9f696366c62cf3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Camera>)))
  "Returns full string definition for message of type '<Camera>"
  (cl:format cl:nil "int32 port~%string address~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Camera)))
  "Returns full string definition for message of type 'Camera"
  (cl:format cl:nil "int32 port~%string address~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Camera>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'address))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Camera>))
  "Converts a ROS message object to a list"
  (cl:list 'Camera
    (cl:cons ':port (port msg))
    (cl:cons ':address (address msg))
))

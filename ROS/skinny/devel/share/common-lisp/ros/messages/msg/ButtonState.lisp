; Auto-generated. Do not edit!


(cl:in-package messages-msg)


;//! \htmlinclude ButtonState.msg.html

(cl:defclass <ButtonState> (roslisp-msg-protocol:ros-message)
  ((joystick
    :reader joystick
    :initarg :joystick
    :type cl:fixnum
    :initform 0)
   (button
    :reader button
    :initarg :button
    :type cl:fixnum
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ButtonState (<ButtonState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ButtonState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ButtonState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name messages-msg:<ButtonState> is deprecated: use messages-msg:ButtonState instead.")))

(cl:ensure-generic-function 'joystick-val :lambda-list '(m))
(cl:defmethod joystick-val ((m <ButtonState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:joystick-val is deprecated.  Use messages-msg:joystick instead.")
  (joystick m))

(cl:ensure-generic-function 'button-val :lambda-list '(m))
(cl:defmethod button-val ((m <ButtonState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:button-val is deprecated.  Use messages-msg:button instead.")
  (button m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <ButtonState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader messages-msg:state-val is deprecated.  Use messages-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ButtonState>) ostream)
  "Serializes a message object of type '<ButtonState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joystick)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'button)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ButtonState>) istream)
  "Deserializes a message object of type '<ButtonState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joystick)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'button)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
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
  "d97fed8babddf36e3ba7a8b9236726be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ButtonState)))
  "Returns md5sum for a message object of type 'ButtonState"
  "d97fed8babddf36e3ba7a8b9236726be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ButtonState>)))
  "Returns full string definition for message of type '<ButtonState>"
  (cl:format cl:nil "uint8 joystick~%uint8 button~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ButtonState)))
  "Returns full string definition for message of type 'ButtonState"
  (cl:format cl:nil "uint8 joystick~%uint8 button~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ButtonState>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ButtonState>))
  "Converts a ROS message object to a list"
  (cl:list 'ButtonState
    (cl:cons ':joystick (joystick msg))
    (cl:cons ':button (button msg))
    (cl:cons ':state (state msg))
))

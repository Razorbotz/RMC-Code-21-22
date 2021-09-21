;; Auto-generated. Do not edit!


(when (boundp 'messages::JoystickCommand)
  (if (not (find-package "MESSAGES"))
    (make-package "MESSAGES"))
  (shadow 'JoystickCommand (find-package "MESSAGES")))
(unless (find-package "MESSAGES::JOYSTICKCOMMAND")
  (make-package "MESSAGES::JOYSTICKCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude JoystickCommand.msg.html


(defclass messages::JoystickCommand
  :super ros::object
  :slots (_axisX _axisY ))

(defmethod messages::JoystickCommand
  (:init
   (&key
    ((:axisX __axisX) 0.0)
    ((:axisY __axisY) 0.0)
    )
   (send-super :init)
   (setq _axisX (float __axisX))
   (setq _axisY (float __axisY))
   self)
  (:axisX
   (&optional __axisX)
   (if __axisX (setq _axisX __axisX)) _axisX)
  (:axisY
   (&optional __axisY)
   (if __axisY (setq _axisY __axisY)) _axisY)
  (:serialization-length
   ()
   (+
    ;; float32 _axisX
    4
    ;; float32 _axisY
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _axisX
       (sys::poke _axisX (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _axisY
       (sys::poke _axisY (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _axisX
     (setq _axisX (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _axisY
     (setq _axisY (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get messages::JoystickCommand :md5sum-) "2ef0f7484f7f056d7fcb586ad36e7d96")
(setf (get messages::JoystickCommand :datatype-) "messages/JoystickCommand")
(setf (get messages::JoystickCommand :definition-)
      "float32 axisX
float32 axisY

")



(provide :messages/JoystickCommand "2ef0f7484f7f056d7fcb586ad36e7d96")



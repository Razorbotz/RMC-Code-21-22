;; Auto-generated. Do not edit!


(when (boundp 'communication::JoystickCommand)
  (if (not (find-package "COMMUNICATION"))
    (make-package "COMMUNICATION"))
  (shadow 'JoystickCommand (find-package "COMMUNICATION")))
(unless (find-package "COMMUNICATION::JOYSTICKCOMMAND")
  (make-package "COMMUNICATION::JOYSTICKCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude JoystickCommand.msg.html


(defclass communication::JoystickCommand
  :super ros::object
  :slots (_axisX _axisY ))

(defmethod communication::JoystickCommand
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

(setf (get communication::JoystickCommand :md5sum-) "2ef0f7484f7f056d7fcb586ad36e7d96")
(setf (get communication::JoystickCommand :datatype-) "communication/JoystickCommand")
(setf (get communication::JoystickCommand :definition-)
      "float32 axisX
float32 axisY

")



(provide :communication/JoystickCommand "2ef0f7484f7f056d7fcb586ad36e7d96")



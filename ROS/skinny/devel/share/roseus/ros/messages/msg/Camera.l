;; Auto-generated. Do not edit!


(when (boundp 'messages::Camera)
  (if (not (find-package "MESSAGES"))
    (make-package "MESSAGES"))
  (shadow 'Camera (find-package "MESSAGES")))
(unless (find-package "MESSAGES::CAMERA")
  (make-package "MESSAGES::CAMERA"))

(in-package "ROS")
;;//! \htmlinclude Camera.msg.html


(defclass messages::Camera
  :super ros::object
  :slots (_port _address ))

(defmethod messages::Camera
  (:init
   (&key
    ((:port __port) 0)
    ((:address __address) "")
    )
   (send-super :init)
   (setq _port (round __port))
   (setq _address (string __address))
   self)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:address
   (&optional __address)
   (if __address (setq _address __address)) _address)
  (:serialization-length
   ()
   (+
    ;; int32 _port
    4
    ;; string _address
    4 (length _address)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _port
       (write-long _port s)
     ;; string _address
       (write-long (length _address) s) (princ _address s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _port
     (setq _port (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _address
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _address (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get messages::Camera :md5sum-) "5962e9b7032210cb9e9f696366c62cf3")
(setf (get messages::Camera :datatype-) "messages/Camera")
(setf (get messages::Camera :definition-)
      "int32 port
string address

")



(provide :messages/Camera "5962e9b7032210cb9e9f696366c62cf3")



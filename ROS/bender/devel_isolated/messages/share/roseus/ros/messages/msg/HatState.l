;; Auto-generated. Do not edit!


(when (boundp 'messages::HatState)
  (if (not (find-package "MESSAGES"))
    (make-package "MESSAGES"))
  (shadow 'HatState (find-package "MESSAGES")))
(unless (find-package "MESSAGES::HATSTATE")
  (make-package "MESSAGES::HATSTATE"))

(in-package "ROS")
;;//! \htmlinclude HatState.msg.html


(defclass messages::HatState
  :super ros::object
  :slots (_state ))

(defmethod messages::HatState
  (:init
   (&key
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _state (round __state))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; int32 _state
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _state
       (write-long _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _state
     (setq _state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get messages::HatState :md5sum-) "7a2f37ef2ba405f0c7a15cc72663d6f0")
(setf (get messages::HatState :datatype-) "messages/HatState")
(setf (get messages::HatState :definition-)
      "int32 state

")



(provide :messages/HatState "7a2f37ef2ba405f0c7a15cc72663d6f0")



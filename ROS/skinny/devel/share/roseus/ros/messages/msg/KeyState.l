;; Auto-generated. Do not edit!


(when (boundp 'messages::KeyState)
  (if (not (find-package "MESSAGES"))
    (make-package "MESSAGES"))
  (shadow 'KeyState (find-package "MESSAGES")))
(unless (find-package "MESSAGES::KEYSTATE")
  (make-package "MESSAGES::KEYSTATE"))

(in-package "ROS")
;;//! \htmlinclude KeyState.msg.html


(defclass messages::KeyState
  :super ros::object
  :slots (_key _state ))

(defmethod messages::KeyState
  (:init
   (&key
    ((:key __key) 0)
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _key (round __key))
   (setq _state (round __state))
   self)
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; uint16 _key
    2
    ;; uint8 _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _key
       (write-word _key s)
     ;; uint8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _key
     (setq _key (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get messages::KeyState :md5sum-) "5b60023c4a21dcdf8c0309517484805f")
(setf (get messages::KeyState :datatype-) "messages/KeyState")
(setf (get messages::KeyState :definition-)
      "uint16 key
uint8 state
")



(provide :messages/KeyState "5b60023c4a21dcdf8c0309517484805f")



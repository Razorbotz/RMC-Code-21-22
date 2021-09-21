;; Auto-generated. Do not edit!


(when (boundp 'messages::Power)
  (if (not (find-package "MESSAGES"))
    (make-package "MESSAGES"))
  (shadow 'Power (find-package "MESSAGES")))
(unless (find-package "MESSAGES::POWER")
  (make-package "MESSAGES::POWER"))

(in-package "ROS")
;;//! \htmlinclude Power.msg.html


(defclass messages::Power
  :super ros::object
  :slots (_voltage _current0 _current1 _current2 _current3 _current4 _current5 _current6 _current7 _current8 _current9 _current10 _current11 _current12 _current13 _current14 _current15 ))

(defmethod messages::Power
  (:init
   (&key
    ((:voltage __voltage) 0.0)
    ((:current0 __current0) 0.0)
    ((:current1 __current1) 0.0)
    ((:current2 __current2) 0.0)
    ((:current3 __current3) 0.0)
    ((:current4 __current4) 0.0)
    ((:current5 __current5) 0.0)
    ((:current6 __current6) 0.0)
    ((:current7 __current7) 0.0)
    ((:current8 __current8) 0.0)
    ((:current9 __current9) 0.0)
    ((:current10 __current10) 0.0)
    ((:current11 __current11) 0.0)
    ((:current12 __current12) 0.0)
    ((:current13 __current13) 0.0)
    ((:current14 __current14) 0.0)
    ((:current15 __current15) 0.0)
    )
   (send-super :init)
   (setq _voltage (float __voltage))
   (setq _current0 (float __current0))
   (setq _current1 (float __current1))
   (setq _current2 (float __current2))
   (setq _current3 (float __current3))
   (setq _current4 (float __current4))
   (setq _current5 (float __current5))
   (setq _current6 (float __current6))
   (setq _current7 (float __current7))
   (setq _current8 (float __current8))
   (setq _current9 (float __current9))
   (setq _current10 (float __current10))
   (setq _current11 (float __current11))
   (setq _current12 (float __current12))
   (setq _current13 (float __current13))
   (setq _current14 (float __current14))
   (setq _current15 (float __current15))
   self)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:current0
   (&optional __current0)
   (if __current0 (setq _current0 __current0)) _current0)
  (:current1
   (&optional __current1)
   (if __current1 (setq _current1 __current1)) _current1)
  (:current2
   (&optional __current2)
   (if __current2 (setq _current2 __current2)) _current2)
  (:current3
   (&optional __current3)
   (if __current3 (setq _current3 __current3)) _current3)
  (:current4
   (&optional __current4)
   (if __current4 (setq _current4 __current4)) _current4)
  (:current5
   (&optional __current5)
   (if __current5 (setq _current5 __current5)) _current5)
  (:current6
   (&optional __current6)
   (if __current6 (setq _current6 __current6)) _current6)
  (:current7
   (&optional __current7)
   (if __current7 (setq _current7 __current7)) _current7)
  (:current8
   (&optional __current8)
   (if __current8 (setq _current8 __current8)) _current8)
  (:current9
   (&optional __current9)
   (if __current9 (setq _current9 __current9)) _current9)
  (:current10
   (&optional __current10)
   (if __current10 (setq _current10 __current10)) _current10)
  (:current11
   (&optional __current11)
   (if __current11 (setq _current11 __current11)) _current11)
  (:current12
   (&optional __current12)
   (if __current12 (setq _current12 __current12)) _current12)
  (:current13
   (&optional __current13)
   (if __current13 (setq _current13 __current13)) _current13)
  (:current14
   (&optional __current14)
   (if __current14 (setq _current14 __current14)) _current14)
  (:current15
   (&optional __current15)
   (if __current15 (setq _current15 __current15)) _current15)
  (:serialization-length
   ()
   (+
    ;; float32 _voltage
    4
    ;; float32 _current0
    4
    ;; float32 _current1
    4
    ;; float32 _current2
    4
    ;; float32 _current3
    4
    ;; float32 _current4
    4
    ;; float32 _current5
    4
    ;; float32 _current6
    4
    ;; float32 _current7
    4
    ;; float32 _current8
    4
    ;; float32 _current9
    4
    ;; float32 _current10
    4
    ;; float32 _current11
    4
    ;; float32 _current12
    4
    ;; float32 _current13
    4
    ;; float32 _current14
    4
    ;; float32 _current15
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current0
       (sys::poke _current0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current1
       (sys::poke _current1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current2
       (sys::poke _current2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current3
       (sys::poke _current3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current4
       (sys::poke _current4 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current5
       (sys::poke _current5 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current6
       (sys::poke _current6 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current7
       (sys::poke _current7 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current8
       (sys::poke _current8 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current9
       (sys::poke _current9 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current10
       (sys::poke _current10 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current11
       (sys::poke _current11 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current12
       (sys::poke _current12 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current13
       (sys::poke _current13 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current14
       (sys::poke _current14 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current15
       (sys::poke _current15 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _voltage
     (setq _voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current0
     (setq _current0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current1
     (setq _current1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current2
     (setq _current2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current3
     (setq _current3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current4
     (setq _current4 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current5
     (setq _current5 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current6
     (setq _current6 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current7
     (setq _current7 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current8
     (setq _current8 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current9
     (setq _current9 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current10
     (setq _current10 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current11
     (setq _current11 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current12
     (setq _current12 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current13
     (setq _current13 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current14
     (setq _current14 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current15
     (setq _current15 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get messages::Power :md5sum-) "a5bf87df7460476e356f8a0cdbf06096")
(setf (get messages::Power :datatype-) "messages/Power")
(setf (get messages::Power :definition-)
      "float32 voltage
float32 current0
float32 current1
float32 current2
float32 current3
float32 current4
float32 current5
float32 current6
float32 current7
float32 current8
float32 current9
float32 current10
float32 current11
float32 current12
float32 current13
float32 current14 
float32 current15

")



(provide :messages/Power "a5bf87df7460476e356f8a0cdbf06096")



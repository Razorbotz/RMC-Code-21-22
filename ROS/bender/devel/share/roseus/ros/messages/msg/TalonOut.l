;; Auto-generated. Do not edit!


(when (boundp 'messages::TalonOut)
  (if (not (find-package "MESSAGES"))
    (make-package "MESSAGES"))
  (shadow 'TalonOut (find-package "MESSAGES")))
(unless (find-package "MESSAGES::TALONOUT")
  (make-package "MESSAGES::TALONOUT"))

(in-package "ROS")
;;//! \htmlinclude TalonOut.msg.html


(defclass messages::TalonOut
  :super ros::object
  :slots (_deviceID _busVoltage _outputCurrent _outputVoltage _outputPercent _temperature _sensorPosition _sensorVelocity _closedLoopError _integralAccumulator _errorDerivative ))

(defmethod messages::TalonOut
  (:init
   (&key
    ((:deviceID __deviceID) 0)
    ((:busVoltage __busVoltage) 0.0)
    ((:outputCurrent __outputCurrent) 0.0)
    ((:outputVoltage __outputVoltage) 0.0)
    ((:outputPercent __outputPercent) 0.0)
    ((:temperature __temperature) 0.0)
    ((:sensorPosition __sensorPosition) 0)
    ((:sensorVelocity __sensorVelocity) 0)
    ((:closedLoopError __closedLoopError) 0)
    ((:integralAccumulator __integralAccumulator) 0)
    ((:errorDerivative __errorDerivative) 0)
    )
   (send-super :init)
   (setq _deviceID (round __deviceID))
   (setq _busVoltage (float __busVoltage))
   (setq _outputCurrent (float __outputCurrent))
   (setq _outputVoltage (float __outputVoltage))
   (setq _outputPercent (float __outputPercent))
   (setq _temperature (float __temperature))
   (setq _sensorPosition (round __sensorPosition))
   (setq _sensorVelocity (round __sensorVelocity))
   (setq _closedLoopError (round __closedLoopError))
   (setq _integralAccumulator (round __integralAccumulator))
   (setq _errorDerivative (round __errorDerivative))
   self)
  (:deviceID
   (&optional __deviceID)
   (if __deviceID (setq _deviceID __deviceID)) _deviceID)
  (:busVoltage
   (&optional __busVoltage)
   (if __busVoltage (setq _busVoltage __busVoltage)) _busVoltage)
  (:outputCurrent
   (&optional __outputCurrent)
   (if __outputCurrent (setq _outputCurrent __outputCurrent)) _outputCurrent)
  (:outputVoltage
   (&optional __outputVoltage)
   (if __outputVoltage (setq _outputVoltage __outputVoltage)) _outputVoltage)
  (:outputPercent
   (&optional __outputPercent)
   (if __outputPercent (setq _outputPercent __outputPercent)) _outputPercent)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:sensorPosition
   (&optional __sensorPosition)
   (if __sensorPosition (setq _sensorPosition __sensorPosition)) _sensorPosition)
  (:sensorVelocity
   (&optional __sensorVelocity)
   (if __sensorVelocity (setq _sensorVelocity __sensorVelocity)) _sensorVelocity)
  (:closedLoopError
   (&optional __closedLoopError)
   (if __closedLoopError (setq _closedLoopError __closedLoopError)) _closedLoopError)
  (:integralAccumulator
   (&optional __integralAccumulator)
   (if __integralAccumulator (setq _integralAccumulator __integralAccumulator)) _integralAccumulator)
  (:errorDerivative
   (&optional __errorDerivative)
   (if __errorDerivative (setq _errorDerivative __errorDerivative)) _errorDerivative)
  (:serialization-length
   ()
   (+
    ;; int32 _deviceID
    4
    ;; float32 _busVoltage
    4
    ;; float32 _outputCurrent
    4
    ;; float32 _outputVoltage
    4
    ;; float32 _outputPercent
    4
    ;; float32 _temperature
    4
    ;; int32 _sensorPosition
    4
    ;; int32 _sensorVelocity
    4
    ;; int32 _closedLoopError
    4
    ;; int32 _integralAccumulator
    4
    ;; int32 _errorDerivative
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _deviceID
       (write-long _deviceID s)
     ;; float32 _busVoltage
       (sys::poke _busVoltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _outputCurrent
       (sys::poke _outputCurrent (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _outputVoltage
       (sys::poke _outputVoltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _outputPercent
       (sys::poke _outputPercent (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _temperature
       (sys::poke _temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _sensorPosition
       (write-long _sensorPosition s)
     ;; int32 _sensorVelocity
       (write-long _sensorVelocity s)
     ;; int32 _closedLoopError
       (write-long _closedLoopError s)
     ;; int32 _integralAccumulator
       (write-long _integralAccumulator s)
     ;; int32 _errorDerivative
       (write-long _errorDerivative s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _deviceID
     (setq _deviceID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _busVoltage
     (setq _busVoltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _outputCurrent
     (setq _outputCurrent (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _outputVoltage
     (setq _outputVoltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _outputPercent
     (setq _outputPercent (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _temperature
     (setq _temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _sensorPosition
     (setq _sensorPosition (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _sensorVelocity
     (setq _sensorVelocity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _closedLoopError
     (setq _closedLoopError (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _integralAccumulator
     (setq _integralAccumulator (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _errorDerivative
     (setq _errorDerivative (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get messages::TalonOut :md5sum-) "d8f6fc5634dde6263b80ec7fc1aa9dee")
(setf (get messages::TalonOut :datatype-) "messages/TalonOut")
(setf (get messages::TalonOut :definition-)
      "int32 deviceID
float32 busVoltage
float32 outputCurrent
float32 outputVoltage
float32 outputPercent
float32 temperature
int32 sensorPosition
int32 sensorVelocity
int32 closedLoopError 
int32 integralAccumulator
int32 errorDerivative

")



(provide :messages/TalonOut "d8f6fc5634dde6263b80ec7fc1aa9dee")



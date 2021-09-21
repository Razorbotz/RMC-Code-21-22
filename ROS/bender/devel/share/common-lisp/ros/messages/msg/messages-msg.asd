
(cl:in-package :asdf)

(defsystem "messages-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ButtonState" :depends-on ("_package_ButtonState"))
    (:file "_package_ButtonState" :depends-on ("_package"))
    (:file "HatState" :depends-on ("_package_HatState"))
    (:file "_package_HatState" :depends-on ("_package"))
    (:file "JoystickCommand" :depends-on ("_package_JoystickCommand"))
    (:file "_package_JoystickCommand" :depends-on ("_package"))
    (:file "Power" :depends-on ("_package_Power"))
    (:file "_package_Power" :depends-on ("_package"))
    (:file "TalonOut" :depends-on ("_package_TalonOut"))
    (:file "_package_TalonOut" :depends-on ("_package"))
    (:file "VictorOut" :depends-on ("_package_VictorOut"))
    (:file "_package_VictorOut" :depends-on ("_package"))
  ))
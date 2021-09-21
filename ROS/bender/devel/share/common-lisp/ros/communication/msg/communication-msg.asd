
(cl:in-package :asdf)

(defsystem "communication-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "JoystickCommand" :depends-on ("_package_JoystickCommand"))
    (:file "_package_JoystickCommand" :depends-on ("_package"))
  ))
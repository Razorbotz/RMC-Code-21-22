
(cl:in-package :asdf)

(defsystem "talon-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TalonOut" :depends-on ("_package_TalonOut"))
    (:file "_package_TalonOut" :depends-on ("_package"))
  ))
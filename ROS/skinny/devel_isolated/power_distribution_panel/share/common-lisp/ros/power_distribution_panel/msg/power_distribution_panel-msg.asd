
(cl:in-package :asdf)

(defsystem "power_distribution_panel-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Power" :depends-on ("_package_Power"))
    (:file "_package_Power" :depends-on ("_package"))
  ))
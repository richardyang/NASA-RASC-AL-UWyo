
(cl:in-package :asdf)

(defsystem "keyboard-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Key" :depends-on ("_package_Key"))
    (:file "_package_Key" :depends-on ("_package"))
  ))
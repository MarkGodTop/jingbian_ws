
(cl:in-package :asdf)

(defsystem "uav_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Takeoff" :depends-on ("_package_Takeoff"))
    (:file "_package_Takeoff" :depends-on ("_package"))
  ))
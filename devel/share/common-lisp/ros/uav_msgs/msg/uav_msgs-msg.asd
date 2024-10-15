
(cl:in-package :asdf)

(defsystem "uav_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AngleRateThrottle" :depends-on ("_package_AngleRateThrottle"))
    (:file "_package_AngleRateThrottle" :depends-on ("_package"))
    (:file "DesiredStates" :depends-on ("_package_DesiredStates"))
    (:file "_package_DesiredStates" :depends-on ("_package"))
  ))
;; Auto-generated. Do not edit!


(when (boundp 'uav_msgs::Takeoff)
  (if (not (find-package "UAV_MSGS"))
    (make-package "UAV_MSGS"))
  (shadow 'Takeoff (find-package "UAV_MSGS")))
(unless (find-package "UAV_MSGS::TAKEOFF")
  (make-package "UAV_MSGS::TAKEOFF"))
(unless (find-package "UAV_MSGS::TAKEOFFREQUEST")
  (make-package "UAV_MSGS::TAKEOFFREQUEST"))
(unless (find-package "UAV_MSGS::TAKEOFFRESPONSE")
  (make-package "UAV_MSGS::TAKEOFFRESPONSE"))

(in-package "ROS")





(defclass uav_msgs::TakeoffRequest
  :super ros::object
  :slots (_waitOnLastTask ))

(defmethod uav_msgs::TakeoffRequest
  (:init
   (&key
    ((:waitOnLastTask __waitOnLastTask) nil)
    )
   (send-super :init)
   (setq _waitOnLastTask __waitOnLastTask)
   self)
  (:waitOnLastTask
   (&optional (__waitOnLastTask :null))
   (if (not (eq __waitOnLastTask :null)) (setq _waitOnLastTask __waitOnLastTask)) _waitOnLastTask)
  (:serialization-length
   ()
   (+
    ;; bool _waitOnLastTask
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _waitOnLastTask
       (if _waitOnLastTask (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _waitOnLastTask
     (setq _waitOnLastTask (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uav_msgs::TakeoffResponse
  :super ros::object
  :slots (_success ))

(defmethod uav_msgs::TakeoffResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uav_msgs::Takeoff
  :super ros::object
  :slots ())

(setf (get uav_msgs::Takeoff :md5sum-) "3a122444608b7d9984b24a10fba10ac1")
(setf (get uav_msgs::Takeoff :datatype-) "uav_msgs/Takeoff")
(setf (get uav_msgs::Takeoff :request) uav_msgs::TakeoffRequest)
(setf (get uav_msgs::Takeoff :response) uav_msgs::TakeoffResponse)

(defmethod uav_msgs::TakeoffRequest
  (:response () (instance uav_msgs::TakeoffResponse :init)))

(setf (get uav_msgs::TakeoffRequest :md5sum-) "3a122444608b7d9984b24a10fba10ac1")
(setf (get uav_msgs::TakeoffRequest :datatype-) "uav_msgs/TakeoffRequest")
(setf (get uav_msgs::TakeoffRequest :definition-)
      "bool waitOnLastTask 
---
bool success
")

(setf (get uav_msgs::TakeoffResponse :md5sum-) "3a122444608b7d9984b24a10fba10ac1")
(setf (get uav_msgs::TakeoffResponse :datatype-) "uav_msgs/TakeoffResponse")
(setf (get uav_msgs::TakeoffResponse :definition-)
      "bool waitOnLastTask 
---
bool success
")



(provide :uav_msgs/Takeoff "3a122444608b7d9984b24a10fba10ac1")



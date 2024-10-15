; Auto-generated. Do not edit!


(cl:in-package uav_msgs-msg)


;//! \htmlinclude DesiredStates.msg.html

(cl:defclass <DesiredStates> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass DesiredStates (<DesiredStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DesiredStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DesiredStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_msgs-msg:<DesiredStates> is deprecated: use uav_msgs-msg:DesiredStates instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DesiredStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_msgs-msg:header-val is deprecated.  Use uav_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <DesiredStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_msgs-msg:position-val is deprecated.  Use uav_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <DesiredStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_msgs-msg:velocity-val is deprecated.  Use uav_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <DesiredStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_msgs-msg:acceleration-val is deprecated.  Use uav_msgs-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <DesiredStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_msgs-msg:yaw-val is deprecated.  Use uav_msgs-msg:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DesiredStates>) ostream)
  "Serializes a message object of type '<DesiredStates>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acceleration) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DesiredStates>) istream)
  "Deserializes a message object of type '<DesiredStates>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acceleration) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DesiredStates>)))
  "Returns string type for a message object of type '<DesiredStates>"
  "uav_msgs/DesiredStates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DesiredStates)))
  "Returns string type for a message object of type 'DesiredStates"
  "uav_msgs/DesiredStates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DesiredStates>)))
  "Returns md5sum for a message object of type '<DesiredStates>"
  "8b4f25ada6233235f373dc2958068bb8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DesiredStates)))
  "Returns md5sum for a message object of type 'DesiredStates"
  "8b4f25ada6233235f373dc2958068bb8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DesiredStates>)))
  "Returns full string definition for message of type '<DesiredStates>"
  (cl:format cl:nil "std_msgs/Header header~%~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%~%float64 yaw~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DesiredStates)))
  "Returns full string definition for message of type 'DesiredStates"
  (cl:format cl:nil "std_msgs/Header header~%~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%~%float64 yaw~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DesiredStates>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acceleration))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DesiredStates>))
  "Converts a ROS message object to a list"
  (cl:list 'DesiredStates
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':yaw (yaw msg))
))

; Auto-generated. Do not edit!


(cl:in-package interactive_walk_planner_new-msg)


;//! \htmlinclude orientation.msg.html

(cl:defclass <orientation> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ori
    :reader ori
    :initarg :ori
    :type cl:float
    :initform 0.0))
)

(cl:defclass orientation (<orientation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <orientation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'orientation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interactive_walk_planner_new-msg:<orientation> is deprecated: use interactive_walk_planner_new-msg:orientation instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <orientation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_walk_planner_new-msg:header-val is deprecated.  Use interactive_walk_planner_new-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ori-val :lambda-list '(m))
(cl:defmethod ori-val ((m <orientation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_walk_planner_new-msg:ori-val is deprecated.  Use interactive_walk_planner_new-msg:ori instead.")
  (ori m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <orientation>) ostream)
  "Serializes a message object of type '<orientation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ori))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <orientation>) istream)
  "Deserializes a message object of type '<orientation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ori) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<orientation>)))
  "Returns string type for a message object of type '<orientation>"
  "interactive_walk_planner_new/orientation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'orientation)))
  "Returns string type for a message object of type 'orientation"
  "interactive_walk_planner_new/orientation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<orientation>)))
  "Returns md5sum for a message object of type '<orientation>"
  "df1f530ad5dc8c63cdf09aec3b9c60b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'orientation)))
  "Returns md5sum for a message object of type 'orientation"
  "df1f530ad5dc8c63cdf09aec3b9c60b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<orientation>)))
  "Returns full string definition for message of type '<orientation>"
  (cl:format cl:nil "Header header~%float32 ori~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'orientation)))
  "Returns full string definition for message of type 'orientation"
  (cl:format cl:nil "Header header~%float32 ori~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <orientation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <orientation>))
  "Converts a ROS message object to a list"
  (cl:list 'orientation
    (cl:cons ':header (header msg))
    (cl:cons ':ori (ori msg))
))

; Auto-generated. Do not edit!


(cl:in-package interactive_walk_planner_new-msg)


;//! \htmlinclude task_mode.msg.html

(cl:defclass <task_mode> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0))
)

(cl:defclass task_mode (<task_mode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <task_mode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'task_mode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interactive_walk_planner_new-msg:<task_mode> is deprecated: use interactive_walk_planner_new-msg:task_mode instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <task_mode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_walk_planner_new-msg:header-val is deprecated.  Use interactive_walk_planner_new-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <task_mode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_walk_planner_new-msg:x-val is deprecated.  Use interactive_walk_planner_new-msg:x instead.")
  (x m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <task_mode>) ostream)
  "Serializes a message object of type '<task_mode>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <task_mode>) istream)
  "Deserializes a message object of type '<task_mode>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<task_mode>)))
  "Returns string type for a message object of type '<task_mode>"
  "interactive_walk_planner_new/task_mode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'task_mode)))
  "Returns string type for a message object of type 'task_mode"
  "interactive_walk_planner_new/task_mode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<task_mode>)))
  "Returns md5sum for a message object of type '<task_mode>"
  "a2b119ca6a13986b0b1e718a6981a87f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'task_mode)))
  "Returns md5sum for a message object of type 'task_mode"
  "a2b119ca6a13986b0b1e718a6981a87f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<task_mode>)))
  "Returns full string definition for message of type '<task_mode>"
  (cl:format cl:nil "Header header~%int32 x~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'task_mode)))
  "Returns full string definition for message of type 'task_mode"
  (cl:format cl:nil "Header header~%int32 x~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <task_mode>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <task_mode>))
  "Converts a ROS message object to a list"
  (cl:list 'task_mode
    (cl:cons ':header (header msg))
    (cl:cons ':x (x msg))
))

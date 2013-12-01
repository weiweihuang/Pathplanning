; Auto-generated. Do not edit!


(cl:in-package interactive_walk_planner_new-msg)


;//! \htmlinclude foot_sequence.msg.html

(cl:defclass <foot_sequence> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (x
    :reader x
    :initarg :x
    :type (cl:vector cl:float)
   :initform (cl:make-array 50 :element-type 'cl:float :initial-element 0.0))
   (y
    :reader y
    :initarg :y
    :type (cl:vector cl:float)
   :initform (cl:make-array 50 :element-type 'cl:float :initial-element 0.0))
   (theta
    :reader theta
    :initarg :theta
    :type (cl:vector cl:float)
   :initform (cl:make-array 50 :element-type 'cl:float :initial-element 0.0))
   (s
    :reader s
    :initarg :s
    :type (cl:vector cl:float)
   :initform (cl:make-array 50 :element-type 'cl:float :initial-element 0.0))
   (height
    :reader height
    :initarg :height
    :type (cl:vector cl:float)
   :initform (cl:make-array 50 :element-type 'cl:float :initial-element 0.0))
   (normal_x
    :reader normal_x
    :initarg :normal_x
    :type (cl:vector cl:float)
   :initform (cl:make-array 50 :element-type 'cl:float :initial-element 0.0))
   (normal_y
    :reader normal_y
    :initarg :normal_y
    :type (cl:vector cl:float)
   :initform (cl:make-array 50 :element-type 'cl:float :initial-element 0.0))
   (normal_z
    :reader normal_z
    :initarg :normal_z
    :type (cl:vector cl:float)
   :initform (cl:make-array 50 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass foot_sequence (<foot_sequence>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <foot_sequence>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'foot_sequence)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interactive_walk_planner_new-msg:<foot_sequence> is deprecated: use interactive_walk_planner_new-msg:foot_sequence instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <foot_sequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_walk_planner_new-msg:header-val is deprecated.  Use interactive_walk_planner_new-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <foot_sequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_walk_planner_new-msg:x-val is deprecated.  Use interactive_walk_planner_new-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <foot_sequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_walk_planner_new-msg:y-val is deprecated.  Use interactive_walk_planner_new-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <foot_sequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_walk_planner_new-msg:theta-val is deprecated.  Use interactive_walk_planner_new-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 's-val :lambda-list '(m))
(cl:defmethod s-val ((m <foot_sequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_walk_planner_new-msg:s-val is deprecated.  Use interactive_walk_planner_new-msg:s instead.")
  (s m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <foot_sequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_walk_planner_new-msg:height-val is deprecated.  Use interactive_walk_planner_new-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'normal_x-val :lambda-list '(m))
(cl:defmethod normal_x-val ((m <foot_sequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_walk_planner_new-msg:normal_x-val is deprecated.  Use interactive_walk_planner_new-msg:normal_x instead.")
  (normal_x m))

(cl:ensure-generic-function 'normal_y-val :lambda-list '(m))
(cl:defmethod normal_y-val ((m <foot_sequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_walk_planner_new-msg:normal_y-val is deprecated.  Use interactive_walk_planner_new-msg:normal_y instead.")
  (normal_y m))

(cl:ensure-generic-function 'normal_z-val :lambda-list '(m))
(cl:defmethod normal_z-val ((m <foot_sequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_walk_planner_new-msg:normal_z-val is deprecated.  Use interactive_walk_planner_new-msg:normal_z instead.")
  (normal_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <foot_sequence>) ostream)
  "Serializes a message object of type '<foot_sequence>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'x))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'y))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'theta))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 's))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'height))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'normal_x))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'normal_y))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'normal_z))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <foot_sequence>) istream)
  "Deserializes a message object of type '<foot_sequence>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'x) (cl:make-array 50))
  (cl:let ((vals (cl:slot-value msg 'x)))
    (cl:dotimes (i 50)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'y) (cl:make-array 50))
  (cl:let ((vals (cl:slot-value msg 'y)))
    (cl:dotimes (i 50)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'theta) (cl:make-array 50))
  (cl:let ((vals (cl:slot-value msg 'theta)))
    (cl:dotimes (i 50)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 's) (cl:make-array 50))
  (cl:let ((vals (cl:slot-value msg 's)))
    (cl:dotimes (i 50)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'height) (cl:make-array 50))
  (cl:let ((vals (cl:slot-value msg 'height)))
    (cl:dotimes (i 50)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'normal_x) (cl:make-array 50))
  (cl:let ((vals (cl:slot-value msg 'normal_x)))
    (cl:dotimes (i 50)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'normal_y) (cl:make-array 50))
  (cl:let ((vals (cl:slot-value msg 'normal_y)))
    (cl:dotimes (i 50)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'normal_z) (cl:make-array 50))
  (cl:let ((vals (cl:slot-value msg 'normal_z)))
    (cl:dotimes (i 50)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<foot_sequence>)))
  "Returns string type for a message object of type '<foot_sequence>"
  "interactive_walk_planner_new/foot_sequence")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'foot_sequence)))
  "Returns string type for a message object of type 'foot_sequence"
  "interactive_walk_planner_new/foot_sequence")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<foot_sequence>)))
  "Returns md5sum for a message object of type '<foot_sequence>"
  "ecabddcf87bfd129f951c6d3f6d4047a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'foot_sequence)))
  "Returns md5sum for a message object of type 'foot_sequence"
  "ecabddcf87bfd129f951c6d3f6d4047a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<foot_sequence>)))
  "Returns full string definition for message of type '<foot_sequence>"
  (cl:format cl:nil "Header header~%float32[50] x~%float32[50] y~%float32[50] theta~%float32[50] s~%float32[50] height~%float32[50] normal_x~%float32[50] normal_y~%float32[50] normal_z~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'foot_sequence)))
  "Returns full string definition for message of type 'foot_sequence"
  (cl:format cl:nil "Header header~%float32[50] x~%float32[50] y~%float32[50] theta~%float32[50] s~%float32[50] height~%float32[50] normal_x~%float32[50] normal_y~%float32[50] normal_z~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <foot_sequence>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'theta) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 's) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'height) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'normal_x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'normal_y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'normal_z) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <foot_sequence>))
  "Converts a ROS message object to a list"
  (cl:list 'foot_sequence
    (cl:cons ':header (header msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':s (s msg))
    (cl:cons ':height (height msg))
    (cl:cons ':normal_x (normal_x msg))
    (cl:cons ':normal_y (normal_y msg))
    (cl:cons ':normal_z (normal_z msg))
))

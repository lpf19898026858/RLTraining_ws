; Auto-generated. Do not edit!


(cl:in-package nlp_drone_control-msg)


;//! \htmlinclude Plan.msg.html

(cl:defclass <Plan> (roslisp-msg-protocol:ros-message)
  ((actions
    :reader actions
    :initarg :actions
    :type (cl:vector nlp_drone_control-msg:Action)
   :initform (cl:make-array 0 :element-type 'nlp_drone_control-msg:Action :initial-element (cl:make-instance 'nlp_drone_control-msg:Action)))
   (replan_mode
    :reader replan_mode
    :initarg :replan_mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Plan (<Plan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Plan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Plan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nlp_drone_control-msg:<Plan> is deprecated: use nlp_drone_control-msg:Plan instead.")))

(cl:ensure-generic-function 'actions-val :lambda-list '(m))
(cl:defmethod actions-val ((m <Plan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nlp_drone_control-msg:actions-val is deprecated.  Use nlp_drone_control-msg:actions instead.")
  (actions m))

(cl:ensure-generic-function 'replan_mode-val :lambda-list '(m))
(cl:defmethod replan_mode-val ((m <Plan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nlp_drone_control-msg:replan_mode-val is deprecated.  Use nlp_drone_control-msg:replan_mode instead.")
  (replan_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Plan>) ostream)
  "Serializes a message object of type '<Plan>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'actions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'actions))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'replan_mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Plan>) istream)
  "Deserializes a message object of type '<Plan>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'actions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'actions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'nlp_drone_control-msg:Action))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'replan_mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Plan>)))
  "Returns string type for a message object of type '<Plan>"
  "nlp_drone_control/Plan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Plan)))
  "Returns string type for a message object of type 'Plan"
  "nlp_drone_control/Plan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Plan>)))
  "Returns md5sum for a message object of type '<Plan>"
  "13f0666e18573937822408eb65c6a4cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Plan)))
  "Returns md5sum for a message object of type 'Plan"
  "13f0666e18573937822408eb65c6a4cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Plan>)))
  "Returns full string definition for message of type '<Plan>"
  (cl:format cl:nil "# Represents a list of tool calls (a full execution plan)~%nlp_drone_control/Action[] actions~%uint8 replan_mode          # 0 = APPEND, 1 = REPLACE~%~%~%================================================================================~%MSG: nlp_drone_control/Action~%# Represents a single tool call produced by the LLM~%string function_name       # e.g. \"takeoff\"~%string arguments_json      # raw JSON string of arguments~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Plan)))
  "Returns full string definition for message of type 'Plan"
  (cl:format cl:nil "# Represents a list of tool calls (a full execution plan)~%nlp_drone_control/Action[] actions~%uint8 replan_mode          # 0 = APPEND, 1 = REPLACE~%~%~%================================================================================~%MSG: nlp_drone_control/Action~%# Represents a single tool call produced by the LLM~%string function_name       # e.g. \"takeoff\"~%string arguments_json      # raw JSON string of arguments~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Plan>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'actions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Plan>))
  "Converts a ROS message object to a list"
  (cl:list 'Plan
    (cl:cons ':actions (actions msg))
    (cl:cons ':replan_mode (replan_mode msg))
))

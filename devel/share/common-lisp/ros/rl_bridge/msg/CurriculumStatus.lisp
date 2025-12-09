; Auto-generated. Do not edit!


(cl:in-package rl_bridge-msg)


;//! \htmlinclude CurriculumStatus.msg.html

(cl:defclass <CurriculumStatus> (roslisp-msg-protocol:ros-message)
  ((lesson_id
    :reader lesson_id
    :initarg :lesson_id
    :type cl:integer
    :initform 0)
   (lesson_name
    :reader lesson_name
    :initarg :lesson_name
    :type cl:string
    :initform "")
   (changed
    :reader changed
    :initarg :changed
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CurriculumStatus (<CurriculumStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CurriculumStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CurriculumStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rl_bridge-msg:<CurriculumStatus> is deprecated: use rl_bridge-msg:CurriculumStatus instead.")))

(cl:ensure-generic-function 'lesson_id-val :lambda-list '(m))
(cl:defmethod lesson_id-val ((m <CurriculumStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:lesson_id-val is deprecated.  Use rl_bridge-msg:lesson_id instead.")
  (lesson_id m))

(cl:ensure-generic-function 'lesson_name-val :lambda-list '(m))
(cl:defmethod lesson_name-val ((m <CurriculumStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:lesson_name-val is deprecated.  Use rl_bridge-msg:lesson_name instead.")
  (lesson_name m))

(cl:ensure-generic-function 'changed-val :lambda-list '(m))
(cl:defmethod changed-val ((m <CurriculumStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:changed-val is deprecated.  Use rl_bridge-msg:changed instead.")
  (changed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CurriculumStatus>) ostream)
  "Serializes a message object of type '<CurriculumStatus>"
  (cl:let* ((signed (cl:slot-value msg 'lesson_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lesson_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lesson_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'changed) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CurriculumStatus>) istream)
  "Deserializes a message object of type '<CurriculumStatus>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lesson_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lesson_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lesson_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'changed) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CurriculumStatus>)))
  "Returns string type for a message object of type '<CurriculumStatus>"
  "rl_bridge/CurriculumStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CurriculumStatus)))
  "Returns string type for a message object of type 'CurriculumStatus"
  "rl_bridge/CurriculumStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CurriculumStatus>)))
  "Returns md5sum for a message object of type '<CurriculumStatus>"
  "eed77d69c41fbfca0645d6c4f9105016")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CurriculumStatus)))
  "Returns md5sum for a message object of type 'CurriculumStatus"
  "eed77d69c41fbfca0645d6c4f9105016")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CurriculumStatus>)))
  "Returns full string definition for message of type '<CurriculumStatus>"
  (cl:format cl:nil "~%int32 lesson_id~%string lesson_name~%bool changed~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CurriculumStatus)))
  "Returns full string definition for message of type 'CurriculumStatus"
  (cl:format cl:nil "~%int32 lesson_id~%string lesson_name~%bool changed~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CurriculumStatus>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'lesson_name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CurriculumStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'CurriculumStatus
    (cl:cons ':lesson_id (lesson_id msg))
    (cl:cons ':lesson_name (lesson_name msg))
    (cl:cons ':changed (changed msg))
))

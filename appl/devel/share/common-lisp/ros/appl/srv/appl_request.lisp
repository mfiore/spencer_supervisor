; Auto-generated. Do not edit!


(cl:in-package appl-srv)


;//! \htmlinclude appl_request-request.msg.html

(cl:defclass <appl_request-request> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:integer
    :initform 0)
   (obs
    :reader obs
    :initarg :obs
    :type cl:string
    :initform "")
   (xstate
    :reader xstate
    :initarg :xstate
    :type cl:string
    :initform ""))
)

(cl:defclass appl_request-request (<appl_request-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <appl_request-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'appl_request-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name appl-srv:<appl_request-request> is deprecated: use appl-srv:appl_request-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <appl_request-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader appl-srv:cmd-val is deprecated.  Use appl-srv:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'obs-val :lambda-list '(m))
(cl:defmethod obs-val ((m <appl_request-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader appl-srv:obs-val is deprecated.  Use appl-srv:obs instead.")
  (obs m))

(cl:ensure-generic-function 'xstate-val :lambda-list '(m))
(cl:defmethod xstate-val ((m <appl_request-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader appl-srv:xstate-val is deprecated.  Use appl-srv:xstate instead.")
  (xstate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <appl_request-request>) ostream)
  "Serializes a message object of type '<appl_request-request>"
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'obs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'obs))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'xstate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'xstate))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <appl_request-request>) istream)
  "Deserializes a message object of type '<appl_request-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'obs) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'obs) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'xstate) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'xstate) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<appl_request-request>)))
  "Returns string type for a service object of type '<appl_request-request>"
  "appl/appl_requestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'appl_request-request)))
  "Returns string type for a service object of type 'appl_request-request"
  "appl/appl_requestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<appl_request-request>)))
  "Returns md5sum for a message object of type '<appl_request-request>"
  "56c6280feb8a8eace7add2e5e377b170")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'appl_request-request)))
  "Returns md5sum for a message object of type 'appl_request-request"
  "56c6280feb8a8eace7add2e5e377b170")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<appl_request-request>)))
  "Returns full string definition for message of type '<appl_request-request>"
  (cl:format cl:nil "int64 cmd~%string obs~%string xstate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'appl_request-request)))
  "Returns full string definition for message of type 'appl_request-request"
  (cl:format cl:nil "int64 cmd~%string obs~%string xstate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <appl_request-request>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'obs))
     4 (cl:length (cl:slot-value msg 'xstate))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <appl_request-request>))
  "Converts a ROS message object to a list"
  (cl:list 'appl_request-request
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':obs (obs msg))
    (cl:cons ':xstate (xstate msg))
))
;//! \htmlinclude appl_request-response.msg.html

(cl:defclass <appl_request-response> (roslisp-msg-protocol:ros-message)
  ((action
    :reader action
    :initarg :action
    :type cl:integer
    :initform 0)
   (ystate
    :reader ystate
    :initarg :ystate
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass appl_request-response (<appl_request-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <appl_request-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'appl_request-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name appl-srv:<appl_request-response> is deprecated: use appl-srv:appl_request-response instead.")))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <appl_request-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader appl-srv:action-val is deprecated.  Use appl-srv:action instead.")
  (action m))

(cl:ensure-generic-function 'ystate-val :lambda-list '(m))
(cl:defmethod ystate-val ((m <appl_request-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader appl-srv:ystate-val is deprecated.  Use appl-srv:ystate instead.")
  (ystate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <appl_request-response>) ostream)
  "Serializes a message object of type '<appl_request-response>"
  (cl:let* ((signed (cl:slot-value msg 'action)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ystate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'ystate))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <appl_request-response>) istream)
  "Deserializes a message object of type '<appl_request-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ystate) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ystate)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<appl_request-response>)))
  "Returns string type for a service object of type '<appl_request-response>"
  "appl/appl_requestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'appl_request-response)))
  "Returns string type for a service object of type 'appl_request-response"
  "appl/appl_requestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<appl_request-response>)))
  "Returns md5sum for a message object of type '<appl_request-response>"
  "56c6280feb8a8eace7add2e5e377b170")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'appl_request-response)))
  "Returns md5sum for a message object of type 'appl_request-response"
  "56c6280feb8a8eace7add2e5e377b170")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<appl_request-response>)))
  "Returns full string definition for message of type '<appl_request-response>"
  (cl:format cl:nil "int64 action~%string[] ystate~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'appl_request-response)))
  "Returns full string definition for message of type 'appl_request-response"
  (cl:format cl:nil "int64 action~%string[] ystate~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <appl_request-response>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ystate) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <appl_request-response>))
  "Converts a ROS message object to a list"
  (cl:list 'appl_request-response
    (cl:cons ':action (action msg))
    (cl:cons ':ystate (ystate msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'appl_request)))
  'appl_request-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'appl_request)))
  'appl_request-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'appl_request)))
  "Returns string type for a service object of type '<appl_request>"
  "appl/appl_request")
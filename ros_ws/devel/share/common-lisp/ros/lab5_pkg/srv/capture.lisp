; Auto-generated. Do not edit!


(cl:in-package lab5_pkg-srv)


;//! \htmlinclude capture-request.msg.html

(cl:defclass <capture-request> (roslisp-msg-protocol:ros-message)
  ((capture
    :reader capture
    :initarg :capture
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass capture-request (<capture-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <capture-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'capture-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lab5_pkg-srv:<capture-request> is deprecated: use lab5_pkg-srv:capture-request instead.")))

(cl:ensure-generic-function 'capture-val :lambda-list '(m))
(cl:defmethod capture-val ((m <capture-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab5_pkg-srv:capture-val is deprecated.  Use lab5_pkg-srv:capture instead.")
  (capture m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <capture-request>) ostream)
  "Serializes a message object of type '<capture-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'capture) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <capture-request>) istream)
  "Deserializes a message object of type '<capture-request>"
    (cl:setf (cl:slot-value msg 'capture) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<capture-request>)))
  "Returns string type for a service object of type '<capture-request>"
  "lab5_pkg/captureRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'capture-request)))
  "Returns string type for a service object of type 'capture-request"
  "lab5_pkg/captureRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<capture-request>)))
  "Returns md5sum for a message object of type '<capture-request>"
  "4fafc27449e87a2606a3a44304a19b34")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'capture-request)))
  "Returns md5sum for a message object of type 'capture-request"
  "4fafc27449e87a2606a3a44304a19b34")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<capture-request>)))
  "Returns full string definition for message of type '<capture-request>"
  (cl:format cl:nil "bool capture~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'capture-request)))
  "Returns full string definition for message of type 'capture-request"
  (cl:format cl:nil "bool capture~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <capture-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <capture-request>))
  "Converts a ROS message object to a list"
  (cl:list 'capture-request
    (cl:cons ':capture (capture msg))
))
;//! \htmlinclude capture-response.msg.html

(cl:defclass <capture-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass capture-response (<capture-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <capture-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'capture-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lab5_pkg-srv:<capture-response> is deprecated: use lab5_pkg-srv:capture-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <capture-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab5_pkg-srv:success-val is deprecated.  Use lab5_pkg-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <capture-response>) ostream)
  "Serializes a message object of type '<capture-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <capture-response>) istream)
  "Deserializes a message object of type '<capture-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<capture-response>)))
  "Returns string type for a service object of type '<capture-response>"
  "lab5_pkg/captureResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'capture-response)))
  "Returns string type for a service object of type 'capture-response"
  "lab5_pkg/captureResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<capture-response>)))
  "Returns md5sum for a message object of type '<capture-response>"
  "4fafc27449e87a2606a3a44304a19b34")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'capture-response)))
  "Returns md5sum for a message object of type 'capture-response"
  "4fafc27449e87a2606a3a44304a19b34")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<capture-response>)))
  "Returns full string definition for message of type '<capture-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'capture-response)))
  "Returns full string definition for message of type 'capture-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <capture-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <capture-response>))
  "Converts a ROS message object to a list"
  (cl:list 'capture-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'capture)))
  'capture-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'capture)))
  'capture-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'capture)))
  "Returns string type for a service object of type '<capture>"
  "lab5_pkg/capture")
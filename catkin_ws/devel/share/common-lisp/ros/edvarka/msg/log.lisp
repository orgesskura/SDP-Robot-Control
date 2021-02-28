; Auto-generated. Do not edit!


(cl:in-package edvarka-msg)


;//! \htmlinclude log.msg.html

(cl:defclass <log> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass log (<log>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <log>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'log)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name edvarka-msg:<log> is deprecated: use edvarka-msg:log instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <log>) ostream)
  "Serializes a message object of type '<log>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <log>) istream)
  "Deserializes a message object of type '<log>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<log>)))
  "Returns string type for a message object of type '<log>"
  "edvarka/log")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'log)))
  "Returns string type for a message object of type 'log"
  "edvarka/log")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<log>)))
  "Returns md5sum for a message object of type '<log>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'log)))
  "Returns md5sum for a message object of type 'log"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<log>)))
  "Returns full string definition for message of type '<log>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'log)))
  "Returns full string definition for message of type 'log"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <log>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <log>))
  "Converts a ROS message object to a list"
  (cl:list 'log
))

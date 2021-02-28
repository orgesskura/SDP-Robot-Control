;; Auto-generated. Do not edit!


(when (boundp 'edvarka::log)
  (if (not (find-package "EDVARKA"))
    (make-package "EDVARKA"))
  (shadow 'log (find-package "EDVARKA")))
(unless (find-package "EDVARKA::LOG")
  (make-package "EDVARKA::LOG"))

(in-package "ROS")
;;//! \htmlinclude log.msg.html


(defclass edvarka::log
  :super ros::object
  :slots ())

(defmethod edvarka::log
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get edvarka::log :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get edvarka::log :datatype-) "edvarka/log")
(setf (get edvarka::log :definition-)
      "
")



(provide :edvarka/log "d41d8cd98f00b204e9800998ecf8427e")



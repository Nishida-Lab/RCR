;; Auto-generated. Do not edit!


(when (boundp 'rcr2018::DcmCommand)
  (if (not (find-package "RCR2018"))
    (make-package "RCR2018"))
  (shadow 'DcmCommand (find-package "RCR2018")))
(unless (find-package "RCR2018::DCMCOMMAND")
  (make-package "RCR2018::DCMCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude DcmCommand.msg.html


(defclass rcr2018::DcmCommand
  :super ros::object
  :slots (_cmd_vel ))

(defmethod rcr2018::DcmCommand
  (:init
   (&key
    ((:cmd_vel __cmd_vel) 0.0)
    )
   (send-super :init)
   (setq _cmd_vel (float __cmd_vel))
   self)
  (:cmd_vel
   (&optional __cmd_vel)
   (if __cmd_vel (setq _cmd_vel __cmd_vel)) _cmd_vel)
  (:serialization-length
   ()
   (+
    ;; float64 _cmd_vel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _cmd_vel
       (sys::poke _cmd_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _cmd_vel
     (setq _cmd_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rcr2018::DcmCommand :md5sum-) "93e14900f3ac3cbfca813e5ff5e2bd6d")
(setf (get rcr2018::DcmCommand :datatype-) "rcr2018/DcmCommand")
(setf (get rcr2018::DcmCommand :definition-)
      "float64 cmd_vel


")



(provide :rcr2018/DcmCommand "93e14900f3ac3cbfca813e5ff5e2bd6d")



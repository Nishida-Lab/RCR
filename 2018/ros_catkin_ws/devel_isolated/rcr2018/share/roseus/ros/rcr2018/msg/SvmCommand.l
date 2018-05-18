;; Auto-generated. Do not edit!


(when (boundp 'rcr2018::SvmCommand)
  (if (not (find-package "RCR2018"))
    (make-package "RCR2018"))
  (shadow 'SvmCommand (find-package "RCR2018")))
(unless (find-package "RCR2018::SVMCOMMAND")
  (make-package "RCR2018::SVMCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude SvmCommand.msg.html


(defclass rcr2018::SvmCommand
  :super ros::object
  :slots (_cmd_ang_vel ))

(defmethod rcr2018::SvmCommand
  (:init
   (&key
    ((:cmd_ang_vel __cmd_ang_vel) 0.0)
    )
   (send-super :init)
   (setq _cmd_ang_vel (float __cmd_ang_vel))
   self)
  (:cmd_ang_vel
   (&optional __cmd_ang_vel)
   (if __cmd_ang_vel (setq _cmd_ang_vel __cmd_ang_vel)) _cmd_ang_vel)
  (:serialization-length
   ()
   (+
    ;; float64 _cmd_ang_vel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _cmd_ang_vel
       (sys::poke _cmd_ang_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _cmd_ang_vel
     (setq _cmd_ang_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rcr2018::SvmCommand :md5sum-) "c6dddac8b1d4dfc1d29704413439e983")
(setf (get rcr2018::SvmCommand :datatype-) "rcr2018/SvmCommand")
(setf (get rcr2018::SvmCommand :definition-)
      "float64 cmd_ang_vel


")



(provide :rcr2018/SvmCommand "c6dddac8b1d4dfc1d29704413439e983")



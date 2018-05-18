;; Auto-generated. Do not edit!


(when (boundp 'rcr2018::AngVel)
  (if (not (find-package "RCR2018"))
    (make-package "RCR2018"))
  (shadow 'AngVel (find-package "RCR2018")))
(unless (find-package "RCR2018::ANGVEL")
  (make-package "RCR2018::ANGVEL"))

(in-package "ROS")
;;//! \htmlinclude AngVel.msg.html


(defclass rcr2018::AngVel
  :super ros::object
  :slots (_ang_vel ))

(defmethod rcr2018::AngVel
  (:init
   (&key
    ((:ang_vel __ang_vel) 0.0)
    )
   (send-super :init)
   (setq _ang_vel (float __ang_vel))
   self)
  (:ang_vel
   (&optional __ang_vel)
   (if __ang_vel (setq _ang_vel __ang_vel)) _ang_vel)
  (:serialization-length
   ()
   (+
    ;; float64 _ang_vel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _ang_vel
       (sys::poke _ang_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _ang_vel
     (setq _ang_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rcr2018::AngVel :md5sum-) "0654aa42e34bb45bcf83b5431e81941d")
(setf (get rcr2018::AngVel :datatype-) "rcr2018/AngVel")
(setf (get rcr2018::AngVel :definition-)
      "float64 ang_vel


")



(provide :rcr2018/AngVel "0654aa42e34bb45bcf83b5431e81941d")



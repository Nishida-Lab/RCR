;; Auto-generated. Do not edit!


(when (boundp 'rcr2018::TofFront)
  (if (not (find-package "RCR2018"))
    (make-package "RCR2018"))
  (shadow 'TofFront (find-package "RCR2018")))
(unless (find-package "RCR2018::TOFFRONT")
  (make-package "RCR2018::TOFFRONT"))

(in-package "ROS")
;;//! \htmlinclude TofFront.msg.html


(defclass rcr2018::TofFront
  :super ros::object
  :slots (_front ))

(defmethod rcr2018::TofFront
  (:init
   (&key
    ((:front __front) 0.0)
    )
   (send-super :init)
   (setq _front (float __front))
   self)
  (:front
   (&optional __front)
   (if __front (setq _front __front)) _front)
  (:serialization-length
   ()
   (+
    ;; float64 _front
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _front
       (sys::poke _front (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _front
     (setq _front (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rcr2018::TofFront :md5sum-) "ea4f19e94f2ce3074d518ff4d026a5c8")
(setf (get rcr2018::TofFront :datatype-) "rcr2018/TofFront")
(setf (get rcr2018::TofFront :definition-)
      "float64 front


")



(provide :rcr2018/TofFront "ea4f19e94f2ce3074d518ff4d026a5c8")



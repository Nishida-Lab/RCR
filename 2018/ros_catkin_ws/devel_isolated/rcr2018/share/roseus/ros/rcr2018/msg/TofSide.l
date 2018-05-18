;; Auto-generated. Do not edit!


(when (boundp 'rcr2018::TofSide)
  (if (not (find-package "RCR2018"))
    (make-package "RCR2018"))
  (shadow 'TofSide (find-package "RCR2018")))
(unless (find-package "RCR2018::TOFSIDE")
  (make-package "RCR2018::TOFSIDE"))

(in-package "ROS")
;;//! \htmlinclude TofSide.msg.html


(defclass rcr2018::TofSide
  :super ros::object
  :slots (_right _left ))

(defmethod rcr2018::TofSide
  (:init
   (&key
    ((:right __right) 0.0)
    ((:left __left) 0.0)
    )
   (send-super :init)
   (setq _right (float __right))
   (setq _left (float __left))
   self)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:serialization-length
   ()
   (+
    ;; float64 _right
    8
    ;; float64 _left
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _right
       (sys::poke _right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left
       (sys::poke _left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _right
     (setq _right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left
     (setq _left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rcr2018::TofSide :md5sum-) "69961745475ae50d2f0b8ca4d0df065b")
(setf (get rcr2018::TofSide :datatype-) "rcr2018/TofSide")
(setf (get rcr2018::TofSide :definition-)
      "float64 right
float64 left


")



(provide :rcr2018/TofSide "69961745475ae50d2f0b8ca4d0df065b")



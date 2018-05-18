;; Auto-generated. Do not edit!


(when (boundp 'rcr2018::LineCount)
  (if (not (find-package "RCR2018"))
    (make-package "RCR2018"))
  (shadow 'LineCount (find-package "RCR2018")))
(unless (find-package "RCR2018::LINECOUNT")
  (make-package "RCR2018::LINECOUNT"))

(in-package "ROS")
;;//! \htmlinclude LineCount.msg.html


(defclass rcr2018::LineCount
  :super ros::object
  :slots (_count ))

(defmethod rcr2018::LineCount
  (:init
   (&key
    ((:count __count) 0)
    )
   (send-super :init)
   (setq _count (round __count))
   self)
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:serialization-length
   ()
   (+
    ;; uint8 _count
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _count
       (write-byte _count s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _count
     (setq _count (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get rcr2018::LineCount :md5sum-) "2fa4e861824f4267d0328df36b408141")
(setf (get rcr2018::LineCount :datatype-) "rcr2018/LineCount")
(setf (get rcr2018::LineCount :definition-)
      "uint8 count


")



(provide :rcr2018/LineCount "2fa4e861824f4267d0328df36b408141")



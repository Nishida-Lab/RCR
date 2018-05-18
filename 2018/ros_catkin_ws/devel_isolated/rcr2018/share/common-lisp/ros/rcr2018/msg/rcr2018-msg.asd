
(cl:in-package :asdf)

(defsystem "rcr2018-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AngVel" :depends-on ("_package_AngVel"))
    (:file "_package_AngVel" :depends-on ("_package"))
    (:file "LineCount" :depends-on ("_package_LineCount"))
    (:file "_package_LineCount" :depends-on ("_package"))
    (:file "TofFront" :depends-on ("_package_TofFront"))
    (:file "_package_TofFront" :depends-on ("_package"))
    (:file "TofSide" :depends-on ("_package_TofSide"))
    (:file "_package_TofSide" :depends-on ("_package"))
  ))
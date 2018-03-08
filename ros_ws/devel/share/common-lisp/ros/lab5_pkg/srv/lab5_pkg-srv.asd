
(cl:in-package :asdf)

(defsystem "lab5_pkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "capture" :depends-on ("_package_capture"))
    (:file "_package_capture" :depends-on ("_package"))
  ))
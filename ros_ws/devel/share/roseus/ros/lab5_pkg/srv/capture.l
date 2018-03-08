;; Auto-generated. Do not edit!


(when (boundp 'lab5_pkg::capture)
  (if (not (find-package "LAB5_PKG"))
    (make-package "LAB5_PKG"))
  (shadow 'capture (find-package "LAB5_PKG")))
(unless (find-package "LAB5_PKG::CAPTURE")
  (make-package "LAB5_PKG::CAPTURE"))
(unless (find-package "LAB5_PKG::CAPTUREREQUEST")
  (make-package "LAB5_PKG::CAPTUREREQUEST"))
(unless (find-package "LAB5_PKG::CAPTURERESPONSE")
  (make-package "LAB5_PKG::CAPTURERESPONSE"))

(in-package "ROS")





(defclass lab5_pkg::captureRequest
  :super ros::object
  :slots (_capture ))

(defmethod lab5_pkg::captureRequest
  (:init
   (&key
    ((:capture __capture) nil)
    )
   (send-super :init)
   (setq _capture __capture)
   self)
  (:capture
   (&optional __capture)
   (if __capture (setq _capture __capture)) _capture)
  (:serialization-length
   ()
   (+
    ;; bool _capture
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _capture
       (if _capture (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _capture
     (setq _capture (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass lab5_pkg::captureResponse
  :super ros::object
  :slots (_success ))

(defmethod lab5_pkg::captureResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass lab5_pkg::capture
  :super ros::object
  :slots ())

(setf (get lab5_pkg::capture :md5sum-) "4fafc27449e87a2606a3a44304a19b34")
(setf (get lab5_pkg::capture :datatype-) "lab5_pkg/capture")
(setf (get lab5_pkg::capture :request) lab5_pkg::captureRequest)
(setf (get lab5_pkg::capture :response) lab5_pkg::captureResponse)

(defmethod lab5_pkg::captureRequest
  (:response () (instance lab5_pkg::captureResponse :init)))

(setf (get lab5_pkg::captureRequest :md5sum-) "4fafc27449e87a2606a3a44304a19b34")
(setf (get lab5_pkg::captureRequest :datatype-) "lab5_pkg/captureRequest")
(setf (get lab5_pkg::captureRequest :definition-)
      "bool capture
---
bool success

")

(setf (get lab5_pkg::captureResponse :md5sum-) "4fafc27449e87a2606a3a44304a19b34")
(setf (get lab5_pkg::captureResponse :datatype-) "lab5_pkg/captureResponse")
(setf (get lab5_pkg::captureResponse :definition-)
      "bool capture
---
bool success

")



(provide :lab5_pkg/capture "4fafc27449e87a2606a3a44304a19b34")



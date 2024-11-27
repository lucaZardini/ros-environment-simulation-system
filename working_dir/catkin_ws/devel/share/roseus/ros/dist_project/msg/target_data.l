;; Auto-generated. Do not edit!


(when (boundp 'dist_project::target_data)
  (if (not (find-package "DIST_PROJECT"))
    (make-package "DIST_PROJECT"))
  (shadow 'target_data (find-package "DIST_PROJECT")))
(unless (find-package "DIST_PROJECT::TARGET_DATA")
  (make-package "DIST_PROJECT::TARGET_DATA"))

(in-package "ROS")
;;//! \htmlinclude target_data.msg.html


(defclass dist_project::target_data
  :super ros::object
  :slots (_robot_id _x _y _z ))

(defmethod dist_project::target_data
  (:init
   (&key
    ((:robot_id __robot_id) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _robot_id (round __robot_id))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:robot_id
   (&optional __robot_id)
   (if __robot_id (setq _robot_id __robot_id)) _robot_id)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; int64 _robot_id
    8
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _robot_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _robot_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _robot_id) (= (length (_robot_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_robot_id . bv) 0) 0) s)
              (write-long (ash (elt (_robot_id . bv) 1) -1) s))
             ((and (class _robot_id) (= (length (_robot_id . bv)) 1)) ;; big1
              (write-long (elt (_robot_id . bv) 0) s)
              (write-long (if (>= _robot_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _robot_id s)(write-long (if (>= _robot_id 0) 0 #xffffffff) s)))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _robot_id
#+(or :alpha :irix6 :x86_64)
      (setf _robot_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _robot_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get dist_project::target_data :md5sum-) "009ba69527b235acb255beb01782a6ac")
(setf (get dist_project::target_data :datatype-) "dist_project/target_data")
(setf (get dist_project::target_data :definition-)
      "int64 robot_id
float64 x
float64 y
float64 z
")



(provide :dist_project/target_data "009ba69527b235acb255beb01782a6ac")



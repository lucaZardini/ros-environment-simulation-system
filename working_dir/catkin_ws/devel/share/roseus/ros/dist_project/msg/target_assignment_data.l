;; Auto-generated. Do not edit!


(when (boundp 'dist_project::target_assignment_data)
  (if (not (find-package "DIST_PROJECT"))
    (make-package "DIST_PROJECT"))
  (shadow 'target_assignment_data (find-package "DIST_PROJECT")))
(unless (find-package "DIST_PROJECT::TARGET_ASSIGNMENT_DATA")
  (make-package "DIST_PROJECT::TARGET_ASSIGNMENT_DATA"))

(in-package "ROS")
;;//! \htmlinclude target_assignment_data.msg.html


(defclass dist_project::target_assignment_data
  :super ros::object
  :slots (_sender_robot_id _assigned_robot_id _x _y _z ))

(defmethod dist_project::target_assignment_data
  (:init
   (&key
    ((:sender_robot_id __sender_robot_id) 0)
    ((:assigned_robot_id __assigned_robot_id) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _sender_robot_id (round __sender_robot_id))
   (setq _assigned_robot_id (round __assigned_robot_id))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:sender_robot_id
   (&optional __sender_robot_id)
   (if __sender_robot_id (setq _sender_robot_id __sender_robot_id)) _sender_robot_id)
  (:assigned_robot_id
   (&optional __assigned_robot_id)
   (if __assigned_robot_id (setq _assigned_robot_id __assigned_robot_id)) _assigned_robot_id)
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
    ;; int64 _sender_robot_id
    8
    ;; int64 _assigned_robot_id
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
     ;; int64 _sender_robot_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _sender_robot_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _sender_robot_id) (= (length (_sender_robot_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_sender_robot_id . bv) 0) 0) s)
              (write-long (ash (elt (_sender_robot_id . bv) 1) -1) s))
             ((and (class _sender_robot_id) (= (length (_sender_robot_id . bv)) 1)) ;; big1
              (write-long (elt (_sender_robot_id . bv) 0) s)
              (write-long (if (>= _sender_robot_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _sender_robot_id s)(write-long (if (>= _sender_robot_id 0) 0 #xffffffff) s)))
     ;; int64 _assigned_robot_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _assigned_robot_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _assigned_robot_id) (= (length (_assigned_robot_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_assigned_robot_id . bv) 0) 0) s)
              (write-long (ash (elt (_assigned_robot_id . bv) 1) -1) s))
             ((and (class _assigned_robot_id) (= (length (_assigned_robot_id . bv)) 1)) ;; big1
              (write-long (elt (_assigned_robot_id . bv) 0) s)
              (write-long (if (>= _assigned_robot_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _assigned_robot_id s)(write-long (if (>= _assigned_robot_id 0) 0 #xffffffff) s)))
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
   ;; int64 _sender_robot_id
#+(or :alpha :irix6 :x86_64)
      (setf _sender_robot_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _sender_robot_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _assigned_robot_id
#+(or :alpha :irix6 :x86_64)
      (setf _assigned_robot_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _assigned_robot_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(setf (get dist_project::target_assignment_data :md5sum-) "e421d61aefbe78f0d6c0c39a64cfb320")
(setf (get dist_project::target_assignment_data :datatype-) "dist_project/target_assignment_data")
(setf (get dist_project::target_assignment_data :definition-)
      "int64 sender_robot_id
int64 assigned_robot_id
float64 x
float64 y
float64 z
")



(provide :dist_project/target_assignment_data "e421d61aefbe78f0d6c0c39a64cfb320")



;; Auto-generated. Do not edit!


(when (boundp 'dist_project::targets_data)
  (if (not (find-package "DIST_PROJECT"))
    (make-package "DIST_PROJECT"))
  (shadow 'targets_data (find-package "DIST_PROJECT")))
(unless (find-package "DIST_PROJECT::TARGETS_DATA")
  (make-package "DIST_PROJECT::TARGETS_DATA"))

(in-package "ROS")
;;//! \htmlinclude targets_data.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass dist_project::targets_data
  :super ros::object
  :slots (_robot_id _points ))

(defmethod dist_project::targets_data
  (:init
   (&key
    ((:robot_id __robot_id) 0)
    ((:points __points) ())
    )
   (send-super :init)
   (setq _robot_id (round __robot_id))
   (setq _points __points)
   self)
  (:robot_id
   (&optional __robot_id)
   (if __robot_id (setq _robot_id __robot_id)) _robot_id)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; int64 _robot_id
    8
    ;; geometry_msgs/Point[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
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
     ;; geometry_msgs/Point[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
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
   ;; geometry_msgs/Point[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get dist_project::targets_data :md5sum-) "65ee33d4860b2ecbe331036c18715b97")
(setf (get dist_project::targets_data :datatype-) "dist_project/targets_data")
(setf (get dist_project::targets_data :definition-)
      "int64 robot_id
geometry_msgs/Point[] points

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :dist_project/targets_data "65ee33d4860b2ecbe331036c18715b97")



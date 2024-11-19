execute_process(COMMAND "/home/marco/shared/working_dir/catkin_ws/build/unique_identifier/unique_id/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/marco/shared/working_dir/catkin_ws/build/unique_identifier/unique_id/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

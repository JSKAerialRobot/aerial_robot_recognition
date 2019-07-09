add_rostest(test-single_color_ground_object_detection.test ARGS contour_area_size:=0 gui:=false) # max size mode
add_rostest(test-single_color_ground_object_detection.test ARGS gui:=false) # size filtering mode

if(roslaunch_VERSION VERSION_LESS "1.11.1")
  message(WARNING "roslaunch_add_file check fails with unsupported doc attributes ${roslaunch_VERSION}")
else()
  file(GLOB LAUNCH_FILES *.test)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
endif()



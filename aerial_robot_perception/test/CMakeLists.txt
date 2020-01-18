add_rostest(test-single_color_ground_object_detection.test ARGS contour_area_size:=0 contour_area_min_thre:=0.04 gui:=false) # max size mode
add_rostest(test-single_color_ground_object_detection.test ARGS gui:=false) # size filtering mode
add_rostest(test-laser_line_extraction.test ARGS gui:=false) # laser line extraction
add_rostest(test-horizontal_brick_detection.test ARGS gui:=false) # horizontal brick detection

if(roslaunch_VERSION VERSION_LESS "1.11.1")
  message(WARNING "roslaunch_add_file check fails with unsupported doc attributes ${roslaunch_VERSION}")
else()
  file(GLOB LAUNCH_FILES *.test)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
endif()



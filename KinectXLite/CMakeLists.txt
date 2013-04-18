project(KINECTXLITE)

#execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/KinectXLite.conf ${CMAKE_BINARY_DIR}/bin/KinectXLite.conf)

set(OpenCV_LIBS opencv_core)
add_library(KinectXLite MODULE KinectXLite.cc)
target_link_libraries(KinectXLite bvs ${OpenCV_LIBS} usb-1.0)

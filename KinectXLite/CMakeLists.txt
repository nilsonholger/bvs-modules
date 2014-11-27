project(KINECTXLITE)

create_symlink(${CMAKE_CURRENT_SOURCE_DIR}/KinectXLite.conf ${CMAKE_BINARY_DIR}/bin/KinectXLite.conf)
add_bvs_module(KinectXLite KinectXLite.cc)

if(NOT BVS_ANDROID_APP)
	target_link_libraries(KinectXLite usb-1.0 opencv_core)
else()
	#target_link_libraries(KinectXLite opencv_java log)
endif()

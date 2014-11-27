project(STEREOCVCUDA)

#create_symlink(${CMAKE_CURRENT_SOURCE_DIR}/StereoCVCUDA.conf ${CMAKE_BINARY_DIR}/bin/StereoCVCUDA.conf)
add_bvs_module(StereoCVCUDA StereoCVCUDA.cc)

if(NOT BVS_ANDROID_APP)
	target_link_libraries(StereoCVCUDA opencv_core opencv_highgui opencv_imgproc opencv_gpu)
else()
	#target_link_libraries(StereoCVCUDA opencv_java log)
endif()

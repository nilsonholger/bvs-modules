project(EXAMPLECV)

#create_symlink(${CMAKE_CURRENT_SOURCE_DIR}/ExampleCV.conf ${CMAKE_BINARY_DIR}/bin/ExampleCV.conf)
add_bvs_module(ExampleCV ExampleCV.cc)

if(NOT BVS_ANDROID_APP)
	target_link_libraries(ExampleCV opencv_core opencv_highgui opencv_imgproc)
else()
	target_link_libraries(ExampleCV opencv_java log)
endif()

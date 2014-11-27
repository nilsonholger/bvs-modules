project(ANDROIDCV)

create_symlink(${CMAKE_CURRENT_SOURCE_DIR}/AndroidCV.conf ${CMAKE_BINARY_DIR}/bin/AndroidCV.conf)
add_bvs_module(AndroidCV AndroidCV.cc)

if(NOT BVS_ANDROID_APP)
	#target_link_libraries(BVSExampleModule ...)
else()
	target_link_libraries(AndroidCV opencv_java log)
endif()

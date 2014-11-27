project(OPENNIXLITE)

create_symlink(${CMAKE_CURRENT_SOURCE_DIR}/OpenNIXLite.conf ${CMAKE_BINARY_DIR}/bin/OpenNIXLite.conf)
add_bvs_module(OpenNIXLite OpenNIXLite.cc)

if(NOT BVS_ANDROID_APP)
	target_link_libraries(OpenNIXLite OpenNI2 opencv_core)
else()
	#target_link_libraries(OpenNIXLite opencv_java log)
endif()

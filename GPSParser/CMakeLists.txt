project(GPSPARSER)

create_symlink(${CMAKE_CURRENT_SOURCE_DIR}/GPSParser.conf ${CMAKE_BINARY_DIR}/bin/GPSParser.conf)
add_bvs_module(GPSParser GPSParser.cc)

if(NOT BVS_ANDROID_APP)
	#target_link_libraries(GPSParser ...)
else()
	#target_link_libraries(GPSParser opencv_java log)
endif()

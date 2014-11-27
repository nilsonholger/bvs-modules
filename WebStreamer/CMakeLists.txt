project(WEBSTREAMER)

create_symlink(${CMAKE_CURRENT_SOURCE_DIR}/WebStreamer.conf ${CMAKE_BINARY_DIR}/bin/WebStreamer.conf)
add_bvs_module(WebStreamer WebStreamer.cc)

if(NOT BVS_ANDROID_APP)
	target_link_libraries(WebStreamer opencv_core turbojpeg boost_system)
else()
	#target_link_libraries(WebStreamer opencv_java log)
endif()

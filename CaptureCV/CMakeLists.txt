project(CAPTURECV)

create_symlink(${CMAKE_CURRENT_SOURCE_DIR}/CaptureCV.conf ${CMAKE_BINARY_DIR}/bin/CaptureCV.conf)
add_bvs_module(CaptureCV CaptureCV.cc)

if(NOT BVS_ANDROID_APP)
	target_link_libraries(CaptureCV opencv_core opencv_highgui opencv_videoio opencv_imgcodecs)
else()
	target_link_libraries(CaptureCV opencv_java log)
endif()


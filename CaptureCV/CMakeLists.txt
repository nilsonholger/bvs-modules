project(CAPTURECV)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/CaptureCV.conf ${CMAKE_BINARY_DIR}/bin/CaptureCV.conf)

set(ENV{BVS_STATIC_MODULES} "$ENV{BVS_STATIC_MODULES};CaptureCV")
add_library(CaptureCV ${BVS_MODULE_TYPE} CaptureCV.cc)
if(BVS_ANDROID_APP)
        set(OpenCV_LIBS opencv_java)
        target_link_libraries(CaptureCV ${BVS_LINK_LIBRARIES} ${OpenCV_LIBS} log)
else()
	set(OpenCV_LIBS opencv_core opencv_highgui)
	target_link_libraries(CaptureCV ${BVS_LINK_LIBRARIES} ${OpenCV_LIBS})
endif()


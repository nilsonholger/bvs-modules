project(CAPTURECV)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/CaptureCV.conf ${CMAKE_BINARY_DIR}/bin/CaptureCV.conf)

add_library(CaptureCV MODULE CaptureCV.cc)
set(OpenCV_LIBS opencv_core opencv_highgui)
target_link_libraries(CaptureCV bvs ${OpenCV_LIBS})


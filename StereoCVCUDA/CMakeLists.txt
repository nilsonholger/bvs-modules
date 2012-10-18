project(STEREOCVCUDA)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/StereoCVCUDAConfig.txt ${CMAKE_BINARY_DIR}/bin/StereoCVCUDAConfig.txt)

add_library(StereoCVCUDA MODULE StereoCVCUDA.cc)
set(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc opencv_gpu)
target_link_libraries(StereoCVCUDA bvs ${OpenCV_LIBS})

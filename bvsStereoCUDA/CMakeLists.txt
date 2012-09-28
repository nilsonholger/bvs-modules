project(BVSSTEREOCUDA)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/bvsStereoCUDAConfig.txt ${CMAKE_BINARY_DIR}/bin/bvsStereoCUDAConfig.txt)

add_library(bvsStereoCUDA MODULE bvsStereoCUDA.cc)
set(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc opencv_gpu)
target_link_libraries(bvsStereoCUDA bvs ${OpenCV_LIBS})

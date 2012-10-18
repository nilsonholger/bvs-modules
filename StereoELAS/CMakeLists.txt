project(STEREOELAS)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/StereoELASConfig.txt ${CMAKE_BINARY_DIR}/bin/StereoELASConfig.txt)

include_directories(SYSTEM elas)
add_library(StereoELAS MODULE StereoELAS.cc elas/descriptor.cpp elas/elas.cpp elas/filter.cpp elas/matrix.cpp elas/triangle.cpp)
add_definitions(-msse3)
disable_compiler_warnings(elas/*.cpp)
set(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc)
target_link_libraries(StereoELAS bvs ${OpenCV_LIBS})

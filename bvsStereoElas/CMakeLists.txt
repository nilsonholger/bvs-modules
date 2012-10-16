project(BVSSTEREOELAS)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/bvsStereoElasConfig.txt ${CMAKE_BINARY_DIR}/bin/bvsStereoElasConfig.txt)

include_directories(SYSTEM elas)
add_library(bvsStereoElas MODULE bvsStereoElas.cc elas/descriptor.cpp elas/elas.cpp elas/filter.cpp elas/matrix.cpp elas/triangle.cpp)
add_definitions(-msse3)
disable_compiler_warnings(elas/*.cpp)
set(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc)
target_link_libraries(bvsStereoElas bvs ${OpenCV_LIBS})

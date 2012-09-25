project(BVSSTEREOELAS)

add_library(bvsStereoElas MODULE bvsStereoElas.cc elas/descriptor.cpp elas/elas.cpp elas/filter.cpp elas/matrix.cpp elas/triangle.cpp)
add_definitions(-msse3)
set(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc)
target_link_libraries(bvsStereoElas bvs ${OpenCV_LIBS})

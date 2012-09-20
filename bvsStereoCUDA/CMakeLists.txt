project(BVSSTEREOCUDA)

add_library(bvsStereoCUDA SHARED bvsStereoCUDA.cc)
set(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc opencv_gpu)
target_link_libraries(bvsStereoCUDA bvs ${OpenCV_LIBS})

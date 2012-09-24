project(BVSCALIBRATION)

add_library(bvsCalibration MODULE bvsCalibration.cc stereocalibration.cc calibrationguide.cc)
set(OpenCV_LIBS opencv_core opencv_highgui opencv_calib3d opencv_features2d opencv_imgproc)
target_link_libraries(bvsCalibration bvs ${OpenCV_LIBS})

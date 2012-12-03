project(CALIBRATIONCV)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationCV.conf ${CMAKE_BINARY_DIR}/bin/CalibrationCV.conf)

add_library(CalibrationCV MODULE CalibrationCV.cc stereocalibration.cc calibrationguide.cc)
set(OpenCV_LIBS opencv_core opencv_highgui opencv_calib3d opencv_features2d opencv_imgproc)
target_link_libraries(CalibrationCV bvs ${OpenCV_LIBS})

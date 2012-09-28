project(BVSCALIBRATION)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/bvsCalibrationConfig.txt ${CMAKE_BINARY_DIR}/bin/bvsCalibrationConfig.txt)

add_library(bvsCalibration MODULE bvsCalibration.cc stereocalibration.cc calibrationguide.cc)
set(OpenCV_LIBS opencv_core opencv_highgui opencv_calib3d opencv_features2d opencv_imgproc)
target_link_libraries(bvsCalibration bvs ${OpenCV_LIBS})

project(CALIBRATIONCV)

create_symlink(${CMAKE_CURRENT_SOURCE_DIR}/CalibrationCV.conf ${CMAKE_BINARY_DIR}/bin/CalibrationCV.conf)
add_bvs_module(CalibrationCV CalibrationCV.cc stereocalibration.cc calibrationguide.cc)

if(NOT BVS_ANDROID_APP)
	target_link_libraries(CalibrationCV opencv_core opencv_highgui opencv_calib3d opencv_features2d opencv_imgproc)
else()
	target_link_libraries(CalibrationCV opencv_java log)
endif()

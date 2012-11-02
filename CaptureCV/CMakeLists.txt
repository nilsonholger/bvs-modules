project(CAPTURECV)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/CaptureCVConfig.txt ${CMAKE_BINARY_DIR}/bin/CaptureCVConfig.txt)

add_library(CaptureCV MODULE CaptureCV.cc)
#set(OpenCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_ml opencv_highgui opencv_objdetect opencv_contrib opencv_legacy opencv_gpu)
set(OpenCV_LIBS opencv_core opencv_highgui )
target_link_libraries(CaptureCV bvs ${OpenCV_LIBS})


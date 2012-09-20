project(BVSCAPTURE)

add_library(bvsCapture SHARED bvsCapture.cc)
#set(OpenCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_ml opencv_highgui opencv_objdetect opencv_contrib opencv_legacy opencv_gpu)
set(OpenCV_LIBS opencv_core opencv_highgui )
target_link_libraries(bvsCapture bvs ${OpenCV_LIBS})


project(EXAMPLECV)

#execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/ExampleCV.conf ${CMAKE_BINARY_DIR}/bin/ExampleCV.conf)

set(ENV{BVS_STATIC_MODULES} "$ENV{BVS_STATIC_MODULES};ExampleCV")
add_library(ExampleCV ${BVS_MODULE_TYPE} ExampleCV.cc)
set(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc)
target_link_libraries(ExampleCV ${BVS_LINK_LIBRARIES} ${OpenCV_LIBS})

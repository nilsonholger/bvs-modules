project(EXAMPLECV)

#execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/ExampleCV.conf ${CMAKE_BINARY_DIR}/bin/ExampleCV.conf)

add_library(ExampleCV MODULE ExampleCV.cc)
set(OpenCV_LIBS opencv_core opencv_highgui)
target_link_libraries(ExampleCV bvs ${OpenCV_LIBS})

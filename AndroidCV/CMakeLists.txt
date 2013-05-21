project(ANDROIDCV)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/AndroidCV.conf ${CMAKE_BINARY_DIR}/bin/AndroidCV.conf)

add_library(AndroidCV MODULE AndroidCV.cc)
target_link_libraries(AndroidCV bvs)

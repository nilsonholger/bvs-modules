project(OPENNIXLITE)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/OpenNIXLite.conf ${CMAKE_BINARY_DIR}/bin/OpenNIXLite.conf)

add_library(OpenNIXLite MODULE OpenNIXLite.cc)
target_link_libraries(OpenNIXLite bvs OpenNI2 opencv_core)

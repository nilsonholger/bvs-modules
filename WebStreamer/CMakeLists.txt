project(WEBSTREAMER)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/WebStreamer.conf ${CMAKE_BINARY_DIR}/bin/WebStreamer.conf)

add_library(WebStreamer MODULE WebStreamer.cc)
target_link_libraries(WebStreamer bvs)

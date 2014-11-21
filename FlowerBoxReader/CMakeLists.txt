project(FLOWERBOXREADER)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/FlowerBoxReader.conf ${CMAKE_BINARY_DIR}/bin/FlowerBoxReader.conf)

set(ENV{BVS_STATIC_MODULES} "$ENV{BVS_STATIC_MODULES};FlowerBoxReader")
add_library(FlowerBoxReader ${BVS_MODULE_TYPE} FlowerBoxReader.cc)
target_link_libraries(FlowerBoxReader ${BVS_LINK_LIBRARIES})

if(NOT BVS_ANDROID_APP)
	target_link_libraries(FlowerBoxReader opencv_core)
else()
	target_link_libraries(FlowerBoxReader opencv_java log)
endif()

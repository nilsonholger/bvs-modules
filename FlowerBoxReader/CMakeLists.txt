project(FLOWERBOXREADER)

create_symlink(${CMAKE_CURRENT_SOURCE_DIR}/FlowerBoxReader.conf ${CMAKE_BINARY_DIR}/bin/FlowerBoxReader.conf)
add_bvs_module(FlowerBoxReader FlowerBoxReader.cc)

if(NOT BVS_ANDROID_APP)
	target_link_libraries(FlowerBoxReader opencv_core opencv_highgui)
else()
	target_link_libraries(FlowerBoxReader opencv_java log)
endif()

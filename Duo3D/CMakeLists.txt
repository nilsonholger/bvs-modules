project(DUO3D)

create_symlink(${CMAKE_CURRENT_SOURCE_DIR}/Duo3D.conf ${CMAKE_BINARY_DIR}/bin/Duo3D.conf)
add_bvs_module(Duo3D ${BVS_MODULE_TYPE} Duo3D.cc)

if(NOT BVS_ANDROID_APP)
	target_link_libraries(Duo3D DUO opencv_core opencv_imgcodecs)
else()
	#target_link_libraries(Duo3D opencv_java log)
endif()

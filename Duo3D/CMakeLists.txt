project(DUO3D)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/Duo3D.conf ${CMAKE_BINARY_DIR}/bin/Duo3D.conf)

set(ENV{BVS_STATIC_MODULES} "$ENV{BVS_STATIC_MODULES};Duo3D")
add_library(Duo3D ${BVS_MODULE_TYPE} Duo3D.cc)
target_link_libraries(Duo3D ${BVS_LINK_LIBRARIES} duo opencv_core)

if(NOT BVS_ANDROID_APP)
	#target_link_libraries(Duo3D ...)
else()
	target_link_libraries(Duo3D opencv_java log)
endif()

project(STEREOELAS)

create_symlink(${CMAKE_CURRENT_SOURCE_DIR}/StereoELAS.conf ${CMAKE_BINARY_DIR}/bin/StereoELAS.conf)
include_directories(SYSTEM elas)
add_bvs_module(StereoELAS StereoELAS.cc elas/descriptor.cpp elas/elas.cpp elas/filter.cpp elas/matrix.cpp elas/triangle.cpp)

add_definitions(-msse3)
disable_compiler_warnings(elas/*.cpp)

if(NOT BVS_ANDROID_APP)
	target_link_libraries(StereoELAS opencv_core opencv_highgui opencv_imgproc)
else()
	#target_link_libraries(StereoELAS opencv_java log)
endif()

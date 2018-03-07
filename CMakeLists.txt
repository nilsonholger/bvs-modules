project(ZEDCAPTURE)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/ZedCapture.conf ${CMAKE_BINARY_DIR}/bin/ZedCapture.conf)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(ZED 2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

set(ENV{BVS_STATIC_MODULES} "$ENV{BVS_STATIC_MODULES};ZedCapture")
add_library(ZedCapture ${BVS_MODULE_TYPE} ZedCapture.cc)
target_link_libraries(ZedCapture ${BVS_LINK_LIBRARIES})

target_link_libraries(ZedCapture ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})

if(NOT BVS_ANDROID_APP)
        target_link_libraries(ZedCapture ${OpenCV_LIBS})
else()
	target_link_libraries(ZedCapture opencv_java log)
endif()

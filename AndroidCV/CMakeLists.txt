project(ANDROIDCV)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/AndroidCV.conf ${CMAKE_BINARY_DIR}/bin/AndroidCV.conf)
add_library(AndroidCV ${BVS_MODULE_TYPE} AndroidCV.cc)

set(ENV{BVS_STATIC_MODULES} "$ENV{BVS_STATIC_MODULES};AndroidCV")
if(BVS_ANDROID_APP)
        set(OpenCV_LIBS -L/home/bsigrist/workspace/opencvsdk/native/libs/armeabi-v7a opencv_java)
        target_link_libraries(AndroidCV ${BVS_LINK_LIBRARIES} ${OpenCV_LIBS} log)
endif()

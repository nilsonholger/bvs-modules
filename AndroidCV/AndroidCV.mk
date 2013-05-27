#AndroidCV

#OpenCV
##### Following line not needed if OpenCV_Manager available on device
#OPENCV_LIB_TYPE:=STATIC
include $(LOCAL_OPENCV)
LOCAL_C_INCLUDES += $(BVS_ROOT_PATH)/lib/include
LOCAL_C_INCLUDES += $(BVS_ROOT_PATH)/android
LOCAL_CPPFLAGS := -std=c++11 -pthread -frtti -fexceptions $(BVS_OPTIONS)
APP_USE_CPP0X := true

LOCAL_ARM_MODE := arm
LOCAL_SRC_FILES := $(BVS_MODULES_PATH)/AndroidCV/AndroidCV.cc

LOCAL_LDLIBS  += -llog
LOCAL_SHARED_LIBRARIES := BvsA
~                                    

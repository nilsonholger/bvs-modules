include $(BVS_ROOT_PATH)/android/local.opencv.mk

include $(LOCAL_OPENCV)

LOCAL_C_INCLUDES += $(BVS_ROOT_PATH)/lib/include
LOCAL_CPPFLAGS := -std=c++11 -pthread -frtti -fexceptions $(BVS_OPTIONS)
APP_USE_CPP0X := true

LOCAL_ARM_MODE := arm
# --llog for logging
LOCAL_LDLIBS  += -llog
LOCAL_SRC_FILES := $(BVS_MODULES_PATH)/ExampleCV/ExampleCV.cc

LOCAL_SHARED_LIBRARIES := BvsA


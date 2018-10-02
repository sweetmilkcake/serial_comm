LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := serial_comm
LOCAL_SRC_FILES := serial_comm.c
LOCAL_CFLAGS := -Wall -Werror -Wno-unused-parameter
include $(BUILD_EXECUTABLE)

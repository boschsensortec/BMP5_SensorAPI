COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= self_test_feature.c

API_LOCATION ?= ../..

C_SRCS += \
$(API_LOCATION)/bmp5.c \
../common/common.c  \
../../source/bmp5_selftest.c \

INCLUDEPATHS += \
$(API_LOCATION) \
../common \
../../source \

include $(COINES_INSTALL_PATH)/coines.mk

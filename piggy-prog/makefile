#
#            DMBS Build System
#     Released into the public domain.
#
#  dean [at] fourwalledcubicle [dot] com
#        www.fourwalledcubicle.com
#
# 2015 Nerd Ralph fork
# nerdralph.blogspot.ca
#

# Run "make help" for target help.

MCU          = atmega328p
F_CPU        = 16000000
OPTIMIZATION = s
TARGET       = piggy-prog
SRC          = $(TARGET).cpp $(wildcard ../libs/*.cpp)
CC_FLAGS     =
LD_FLAGS     =

# Default target
all:

# Include DMBS build script makefiles
DMBS_PATH   ?= ../dmbs/DMBS
include $(DMBS_PATH)/core.mk
include $(DMBS_PATH)/gcc.mk
include $(DMBS_PATH)/cppcheck.mk
include $(DMBS_PATH)/avrdude.mk

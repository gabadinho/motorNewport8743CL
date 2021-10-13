TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += newport8743

# install newport8743.dbd into <top>/dbd
DBD += newport8743.dbd

INC += Newport8743MotorDriver.h

# specify all source files to be compiled and added to the library
newport8743_SRCS += Newport8743MotorDriver.cpp

newport8743_LIBS += motor
newport8743_LIBS += asyn

newport8743_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


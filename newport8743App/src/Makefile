TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += newport8743

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install newport8743.dbd into <top>/dbd
DBD += newport8743.dbd

# specify all source files to be compiled and added to the library
#newport8743_SRCS += xxx

newport8743_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


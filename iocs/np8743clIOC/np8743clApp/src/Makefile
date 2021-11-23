TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = np8743cl
# np8743cl.dbd will be created and installed
DBD += np8743cl.dbd

# np8743cl.dbd will be made up from these files:
np8743cl_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
np8743cl_DBD += asyn.dbd
np8743cl_DBD += drvAsynSerialPort.dbd
np8743cl_DBD += drvAsynIPPort.dbd
#endif
np8743cl_DBD += motorSupport.dbd
np8743cl_DBD += newport8743.dbd

# Add all the support libraries needed by this IOC
np8743cl_LIBS += newport8743
np8743cl_LIBS += motor
#ifdef ASYN
np8743cl_LIBS += asyn
#endif
#ifdef SNCSEQ
np8743cl_LIBS += seq pv
#endif

# np8743cl_registerRecordDeviceDriver.cpp derives from np8743cl.dbd
np8743cl_SRCS += np8743cl_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
np8743cl_SRCS_DEFAULT += np8743clMain.cpp
np8743cl_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#np8743cl_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
np8743cl_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


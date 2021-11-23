# Newport 8743-CL controller support

# Load motor record
dbLoadTemplate("np8743cl.substitutions")

# Configure asyn IP address
drvAsynIPPortConfigure("NPCTRL", "192.168.100.10:4000")

# Load asyn record
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=NEWPORT:, R=ASYN1, PORT=NPCTRL, ADDR=0, OMAX=256, IMAX=256")

# Turn on asyn trace
asynSetTraceMask("NPCTRL", 0, 0x03)
asynSetTraceIOMask("NPCTRL", 0, 0x04)

# Newport8743CreateController(portName, asynPort, numAxes, movingPollingRate, idlePollingRate)
Newport8743CreateController("NP8743CL", "NPCTRL", 2, 50, 200)

# Turn off asyn trace
asynSetTraceMask("NPCTRL", 0, 0x01)
asynSetTraceIOMask("NPCTRL", 0, 0x00)

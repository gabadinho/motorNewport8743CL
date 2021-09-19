#!../../bin/linux-x86_64/np8743cl

#- You may have to change np8743cl to something else
#- everywhere it appears in this file

#< envPaths

## Register all support components
dbLoadDatabase("../../dbd/np8743cl.dbd",0,0)
np8743cl_registerRecordDeviceDriver(pdbbase) 

## Load record instances
dbLoadRecords("../../db/np8743cl.db","user=gabadinho")

iocInit()

## Start any sequence programs
#seq sncnp8743cl,"user=gabadinho"

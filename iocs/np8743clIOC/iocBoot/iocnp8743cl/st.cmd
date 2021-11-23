#!../../bin/linux-x86_64/np8743cl

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/np8743cl.dbd"
np8743cl_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

##
< np8743cl.cmd

iocInit

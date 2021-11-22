/*
FILENAME...   Newport8743MotorDriver.cpp
USAGE...      Motor driver support (model 3, asyn) for the Newport 8743-CL controller

Jose G.C. Gabadinho
September 2021
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "Newport8743MotorDriver.h"

#include <iocsh.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>
#include <epicsThread.h>



#define RECONNECT_SLEEP 0.1
#define SHORTWAIT_SLEEP 0.1



static const char *driverName = "Newport8743CL";



/** Creates a new Newport8743Controller object.
  *
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] asynPortName      The name of the drvAsynIPPPort that was created previously to connect to the Newport 8743-CL controller
  * \param[in] numAxes           The number of axes that this controller supports (discarded and overwritten to 2)
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
Newport8743Controller::Newport8743Controller(const char *portName, const char *asynPortName, int numAxes, double movingPollPeriod, double idlePollPeriod)
    :asynMotorController(portName, 2, NUM_NEWPORT8743_PARAMS, 
                         0,
                         0,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, /* autoconnect */
                         0, 0), /* Default priority and stack size */
                        disconnectedFlag(true) {
    int axis;
    asynStatus status;
    char eos[10];
    int eos_len;
    static const char *functionName = "Newport8743Controller";

    createParam(AXIS_RDBD_PARAMNAME, asynParamFloat64, &driverRetryDeadband);
    createParam(AXIS_MRES_PARAMNAME, asynParamFloat64, &driverMotorRecResolution);
    createParam(AXIS_HOMRTYPE_PARAMNAME, asynParamInt32, &driverHomeReverseType);
    createParam(AXIS_HOMFTYPE_PARAMNAME, asynParamInt32, &driverHomeForwardType);
    createParam(CTRL_RST_PARAMNAME,  asynParamInt32, &driverResetController);

    numAxes = 2; // Force two-axes regardless of what user says

    // Connect to Newport 8743-CL controller
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Creating Newport 8743-CL controller %s to asyn %s with %d axes\n", driverName, functionName, portName, asynPortName, numAxes);
    status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Cannot connect to Newport 8743-CL controller at asyn %s\n", driverName, functionName, asynPortName);
    }

    pasynOctetSyncIO->getInputEos(pasynUserController_, eos, 10, &eos_len);
    if (!eos_len) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Setting input acknowledgement of %s to CR LF\n", driverName, functionName, portName);
        pasynOctetSyncIO->setInputEos(pasynUserController_, "\r\n", 2);
    }

    pasynOctetSyncIO->getOutputEos(pasynUserController_, eos, 10, &eos_len);
    if (!eos_len) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Setting output acknowledgement of %s to CR LF\n", driverName, functionName, portName);
        pasynOctetSyncIO->setOutputEos(pasynUserController_, "\r\n", 2);
    }

    // Create the axis objects
    for (axis=0; axis<numAxes; axis++) {
        new Newport8743Axis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is motorStop_ then it calls pAxis->stop().
  * If the function is motorUpdateStatus_ then it does a poll and forces a callback.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * Motor drivers will reimplement this function if they support 
  * controller-specific parameters on the asynInt32 interface. They should call this
  * base class method for any parameters that are not controller-specific.
  *
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write.
  *
  * \return Result of callParamCallbacks() call or asynMotorController::writeInt32()
  */
asynStatus Newport8743Controller::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    asynMotorAxis *p_axis;
    asynStatus status = asynSuccess;
    const char* functionName = "writeInt32";

    p_axis = getAxis(pasynUser);
    if (p_axis) {
        if (function == driverResetController) {
            p_axis->setIntegerParam(function, value);

            sprintf(this->outString_, CTRL_RESET_CMD);
            writeController();

            status = p_axis->callParamCallbacks();
        } else {
            status = asynMotorController::writeInt32(pasynUser, value);
        }
    } else {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Unable to retrieve Newport 8743-CL %s axis from asynUser in %s\n", this->portName, functionName);
        status = asynError;
    }

    return status;
}

/** Reports on status of the driver.
  * If level > 0 then error message, controller version is printed.
  *
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  */
void Newport8743Controller::report(FILE *fp, int level) {
    asynStatus status = asynError;

    fprintf(fp, "Newport 8743-CL motor controller %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);
    if (level > 0) {
        // Retrieve controller identification
        buildGenericGetCommand(this->outString_, CTRL_ID_CMD);
        status = writeReadController();
        if (status == asynSuccess) {
            fprintf(fp, "  id = %s\n", this->inString_);
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Unable to retrieve Newport 8743-CL %s controller version!\n", this->portName);
            fprintf(fp, "  id = <unknown>\n");
        }

        // Retrieve controller identification
        buildGenericGetCommand(this->outString_, CTRL_ERRORMESSAGE_CMD);
        status = writeReadController();
        if (status == asynSuccess) {
            fprintf(fp, "  msg = %s\n", this->inString_);
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Unable to retrieve Newport 8743-CL %s queued message!\n", this->portName);
            fprintf(fp, "  msg = <unknown>\n");
        }
    }

    // Call the base class method
    asynMotorController::report(fp, level);
}

/** Returns a pointer to an Newport8743Axis object.
  *
  * \param[in] pasynUser asynUser structure that encodes the axis index number
  *
  * \return Newport8743Axis object or NULL if the axis number encoded in pasynUser is invalid
  */
Newport8743Axis* Newport8743Controller::getAxis(asynUser *pasynUser) {
    return static_cast<Newport8743Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an Newport8743Axis object.
  *
  * \param[in] axisNo Axis index number
  *
  * \return Newport8743Axis object or NULL if the axis number encoded in pasynUser is invalid
  */
Newport8743Axis* Newport8743Controller::getAxis(int axisNo) {
    return static_cast<Newport8743Axis*>(asynMotorController::getAxis(axisNo));
}

/** Polls the controller.
  * Reads the joint axes state and updates them.
  *
  * \return Result of writeReadController() call
  */
asynStatus Newport8743Controller::poll() {
    asynStatus status = asynError, final_status = asynSuccess;
    int all_switches = 0x1B; // Simulate activation of both limits switches on both axes
    int error_code = 0; // No error
    Newport8743Axis *axis;

    buildGenericGetCommand(this->outString_, CTRL_STATUS_CMD);
    status = writeReadController();
    if (status == asynSuccess) {
        all_switches = atoi(inString_);
    } else if (status == asynTimeout) {
        return gotDisconnected(); // Beware: return in the middle of the function's code
    } else {
        final_status = asynError;
    }

    if (this->disconnectedFlag) {
        gotConnected();
    }

    if (final_status == asynSuccess) {
        for (int i=0; i<numAxes_; i++) {
            axis = getAxis(i);
            if (axis) {
                axis->updateAxisStatus(all_switches >> (i*3));
            }
        }
    }

    return final_status;
}

/** All the following methods parse a reply sent by the controller.
  *
  */
bool Newport8743Controller::updateControllerStatus(asynStatus status, const char *reply, int& ctrl_stat, asynStatus *asyn_error) {
    bool res = false;
    if ((status == asynSuccess) && (isdigit(*reply))) {
        ctrl_stat = atoi(reply);
        res = true;
    } else {
        if (asyn_error) *asyn_error = asynError;
    }
    return res;
}

/** The following methods generate a command string to be sent to the controller.
  *
  */
bool Newport8743Controller::buildGenericGetCommand(char *buffer, const char *command_format) {
    if ((!buffer) || (!command_format)) {
        return false;
    }
    sprintf(buffer, command_format);
    return true;
}

/** Called by the poll() method after successfully polling the device,
  * either for the first time of after getting timeout errors.
  *
  */
void Newport8743Controller::gotConnected() {
    Newport8743Axis *axis;

    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Successful poll() for Newport 8743-CL %s controller\n", this->portName);
    this->disconnectedFlag = false;

    for (int i=0; i<numAxes_; i++) {
        axis = getAxis(i);
        if (axis) {
            axis->gotConnected();
        }
    }
}

/** Called by the poll() method if it gets a timeout error when polling the device.
  *
  */
asynStatus Newport8743Controller::gotDisconnected() {
    Newport8743Axis *axis;

    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Got a timeout in Newport 8743-CL %s controller, attempting to reconnect...\n", this->portName);
    this->disconnectedFlag = true;

    for (int i=0; i<numAxes_; i++) {
        axis = getAxis(i);
        if (axis) {
            axis->gotDisconnected();
        }
    }

    disconnect(this->pasynUserController_);
    epicsThreadSleep(RECONNECT_SLEEP);
    connect(this->pasynUserController_);

    return asynTimeout;
}



// These are the Newport8743Axis methods

/** Creates a new Newport8743Axis object.
  *
  * \param[in] pC Pointer to the Newport8743Controller to which this axis belongs
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1
  */
Newport8743Axis::Newport8743Axis(Newport8743Controller *pC, int axisNo): asynMotorAxis(pC, axisNo),
    pC_(pC), disconnectedFlag(true) {
    this->motorType = NO_MOTOR;
    this->axisStatus = 3; // Simulate activation of both limits switches
    this->motionDone = false;

    setIntegerParam(pC_->motorStatusHomed_, 0);
    setStatusProblem(asynSuccess);

    callParamCallbacks();
}

/** Reports on status of the axis.
  * If level > 0 then detailed axis information (type, configuration, motion status, etc.) is printed.
  *
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  */
void Newport8743Axis::report(FILE *fp, int level) {
    int hardware_cfg;
    bool closed_loop;
    long deadband;

    if (level > 0) {
        buildGenericGetCommand(pC_->outString_, AXIS_HARDWARECFG_CMD, this->axisNo_);
        if (pC_->writeReadController() == asynSuccess) {
            hardware_cfg = atoi(pC_->inString_);
        } else {
            hardware_cfg = -1;
        }

        closed_loop = isClosedLoop();
        deadband = getDeadband();

        fprintf(fp,
            "  axis %d\n"
            "  type = %d\n"
            "  hardware cfg.= %d\n"
            "  motion done = %d\n"
            "  closed loop = %d\n"
            "  switches state = %d\n"
            "  last deadband = %ld\n",
            this->axisNo_,
            this->motorType,
            hardware_cfg,
            this->motionDone,
            closed_loop,
            this->axisStatus,
            deadband
        );

    } else {
       fprintf(fp,
            "  axis %d\n",
            this->axisNo_);
    }

    asynMotorAxis::report(fp, level);
}

/** Moves the axis to a different target position.
  * Warning: stops any ongoing move or home actions!
  *
  * \param[in] position      The desired target position
  * \param[in] relative      1 for relative position
  * \param[in] minVelocity   Motion parameter
  * \param[in] maxVelocity   Motion parameter
  * \param[in] acceleration  Motion parameter
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus Newport8743Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration) {
    asynStatus status = asynSuccess;
    long target = (long)position;
    long speed = (long)maxVelocity;
    long allowed_error;
    double rdbd, mres;

    if (this->disconnectedFlag) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Newport 8743-CL %s axis %d is disconnected\n", pC_->portName, this->axisNo_);
        return asynTimeout;
    }
    if ((this->motorType != TINY_MOTOR) && (this->motorType != STANDARD_MOTOR)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Unsupported Newport 8743-CL %s axis %d type %d (must be tiny or standard)\n", pC_->portName, this->axisNo_, this->motorType);
        return asynError;
    }

    if (!this->motionDone) {
        status = stopMotor();
        shortWait();
    }

    if ((this->motorType == TINY_MOTOR) && (speed > TINY_MAX_VEL)) {
        speed = TINY_MAX_VEL;
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Lowering velocity to %ld on Newport 8743-CL %s axis %d\n", speed, pC_->portName, this->axisNo_);
    }

    if (status == asynSuccess) {
        pC_->getDoubleParam(this->axisNo_, pC_->driverRetryDeadband, &rdbd);
        pC_->getDoubleParam(this->axisNo_, pC_->driverMotorRecResolution, &mres);
        allowed_error = (long)(rdbd/mres);

        setIntegerParam(pC_->motorStatusDone_, 0);
        if (relative) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Moving Newport 8743-CL %s axis %d to relative %ld at velocity %ld deadband %ld\n", pC_->portName, this->axisNo_, target, speed, allowed_error);
            buildMoveRelativeCommand(pC_->outString_, this->axisNo_, target, speed, allowed_error);
            status = pC_->writeController();
        } else {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Moving Newport 8743-CL %s axis %d to %ld at velocity %ld deadband %ld\n", pC_->portName, this->axisNo_, target, speed, allowed_error);
            buildMoveAbsoluteCommand(pC_->outString_, this->axisNo_, target, speed, allowed_error);
            status = pC_->writeController();
        }

        if (status != asynSuccess) {
            setIntegerParam(pC_->motorStatusDone_, 1);
        } else {
            setIntegerParam(pC_->motorClosedLoop_, 1);
        }
    }

    setStatusProblem(status);

    return callParamCallbacks();
}

/** Starts the axis homing procedure.
  * Warning: stops any ongoing movement!
  *
  * \param[in] minVelocity   Motion parameter
  * \param[in] maxVelocity   Motion parameter
  * \param[in] acceleration  Motion parameter
  * \param[in] forwards      1 if user wants to home forward, 0 for reverse
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus Newport8743Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
    asynStatus status = asynSuccess;
    int hom_type;

    if (this->disconnectedFlag) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Newport 8743-CL %s axis %d is disconnected\n", pC_->portName, this->axisNo_);
        return asynTimeout;
    }
    if ((this->motorType != TINY_MOTOR) && (this->motorType != STANDARD_MOTOR)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Unsupported Newport 8743-CL %s axis %d type %d (must be tiny or standard)\n", pC_->portName, this->axisNo_, this->motorType);
        return asynError;
    }

    if (!this->motionDone) {
        status = stopMotor();
        shortWait();
    }

    if (status == asynSuccess) {
        if (forwards) {
            pC_->getIntegerParam(this->axisNo_, pC_->driverHomeForwardType, &hom_type);
        } else {
            pC_->getIntegerParam(this->axisNo_, pC_->driverHomeReverseType, &hom_type);
        }

        if (hom_type > DISABLE) {
            setIntegerParam(pC_->motorStatusHome_, 1);
            setIntegerParam(pC_->motorStatusHomed_, 0);

            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Homing Newport 8743-CL %s axis %d\n", pC_->portName, this->axisNo_);
            buildHomeCommand(pC_->outString_, this->axisNo_, forwards, static_cast<newportHomeType>(hom_type));
            status = pC_->writeController();
            if (status != asynSuccess) {
                setIntegerParam(pC_->motorStatusHome_, 0);
            }
        } else {
            if (forwards) {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Forward-homing of Newport 8743-CL %s axis %d is disabled!\n", pC_->portName, this->axisNo_);
            } else {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Reverse-homing of Newport 8743-CL %s axis %d is disabled!\n", pC_->portName, this->axisNo_);
            }
            status = asynError;
        }
    }

    setStatusProblem(status);

    return callParamCallbacks();
}

/** Stops an ongoing motion.
  *
  * \param[in] acceleration  Motion parameter
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus Newport8743Axis::stop(double acceleration) {
    asynStatus status = asynError;

    status = stopMotor();
    setStatusProblem(status);

    return callParamCallbacks();
}

/** Forces the axis readback position to some value.
  * Warning: if motor is moving nothing is done!
  *
  * \param[in] position  The desired readback position
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus Newport8743Axis::setPosition(double position) {
    asynStatus status = asynError;

    if ((this->motorType == TINY_MOTOR) || (this->motorType == STANDARD_MOTOR)) {
        if (!this->motionDone) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Due to ongoing motion of Newport 8743-CL %s axis %d, readback position will not be overriden!\n", pC_->portName, this->axisNo_);
        } else {
            buildSetPositionCommand(pC_->outString_, this->axisNo_, position, this->positionReadback);
            status = pC_->writeController();
        }
    } else {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Unsupported Newport 8743-CL %s axis %d type %d (must be tiny or standard)\n", pC_->portName, this->axisNo_, this->motorType);
    }

    setStatusProblem(status);

    return callParamCallbacks();
}

/** Polls the axis.
  * Reads the states, limits, readback, etc. and calls setIntegerParam() or setDoubleParam() for each item that it polls.
  * Waits until all states are successfully received before updating the axis object, to avoid the problem of
  * disconnections between commands.
  *
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0)
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus Newport8743Axis::poll(bool *moving) { 
    asynStatus status = asynError, final_status = asynSuccess;
    int at_limit, motion_done=0, homing;
    bool valid_motion_done = false, valid_readback = false;
    long readback;

    if (this->disconnectedFlag) {
        return asynTimeout;
    }
    if ((this->motorType != TINY_MOTOR) && (this->motorType != STANDARD_MOTOR)) {
        return asynError;
    }

    buildGenericGetCommand(pC_->outString_, AXIS_MOTIONDONE_CMD, this->axisNo_);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        valid_motion_done = true;
        motion_done = atoi(pC_->inString_);
    } else if (status == asynTimeout) {
        return status; // Beware: return in the middle of the function's code
    } else {
        final_status = asynError;
    }

    buildGenericGetCommand(pC_->outString_, AXIS_READBACKPOS_CMD, this->axisNo_);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        valid_readback = true;
        readback = atol(pC_->inString_);
    } else if (status == asynTimeout) {
        return status; // Beware: return in the middle of the function's code
    } else {
        final_status = asynError;
    }

    if (valid_readback) {
        setDoubleParam(pC_->motorEncoderPosition_, readback);
        setDoubleParam(pC_->motorPosition_, readback);
    }

    if (valid_motion_done) {
        if (motion_done) {
            pC_->getIntegerParam(this->axisNo_, pC_->motorStatusHome_, &homing);
            if (homing) {
                setIntegerParam(pC_->motorStatusHome_, 0);
                buildZeroCommand(pC_->outString_, this->axisNo_);
                status = pC_->writeController();
                if (status == asynSuccess) {
                    this->setIntegerParam(pC_->motorStatusHomed_, 1);
                } else {
                    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Unable to set readback to 0 after homing Newport 8743-CL %s axis %d\n", pC_->portName, this->axisNo_);
                    final_status = asynError;
                }
            }
        }

        pC_->getIntegerParam(this->axisNo_, pC_->motorStatusLowLimit_, &at_limit);
        if ((motion_done) && (this->axisStatus&AXIS_NEG_LS) && (!at_limit)) {
            setIntegerParam(pC_->motorStatusLowLimit_, 1);
        } else if ((motion_done) && !(this->axisStatus&AXIS_NEG_LS) && (at_limit)) {
            setIntegerParam(pC_->motorStatusLowLimit_, 0);
        }

        pC_->getIntegerParam(this->axisNo_, pC_->motorStatusHighLimit_, &at_limit);
        if ((motion_done) && (this->axisStatus&AXIS_POS_LS) && (!at_limit)) {
            setIntegerParam(pC_->motorStatusHighLimit_, 1);
        } else if ((motion_done) && !(this->axisStatus&AXIS_POS_LS) && (at_limit)) {
            setIntegerParam(pC_->motorStatusHighLimit_, 0);
        }

        setIntegerParam(pC_->motorStatusDone_, motion_done);
        *moving = !motion_done;
    }

    setStatusProblem(final_status);

    return callParamCallbacks();
}

/** Raises the motor record problem status.
  *
  * \param[in] status Last operation status
  */
void Newport8743Axis::setStatusProblem(asynStatus status) {
    int status_problem;

    pC_->getIntegerParam(this->axisNo_, pC_->motorStatus_, &status_problem);
    if ((status != asynSuccess) && (!status_problem)) {
        setIntegerParam(pC_->motorStatusProblem_, 1);
    }
    if ((status == asynSuccess) && (status_problem)) {
        setIntegerParam(pC_->motorStatusProblem_, 0);
    }
}

/** Updates the axis status, from querying the controller.
  *
  * \param[in] axis_status Axis status
  *                        Bit 0 = positive limit-switch state
  *                        Bit 1 = negative limit-switch state
  *                        Bit 2 = at-home switch state
  */
void Newport8743Axis::updateAxisStatus(int axis_status) {
    this->axisStatus = axis_status;
}

/** Stops a motion.
  *
  * \return Result of writeController() call
  */
asynStatus Newport8743Axis::stopMotor() {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Stopping motion on Newport 8743-CL %s axis %d\n", pC_->portName, this->axisNo_);
    buildStopCommand(pC_->outString_, this->axisNo_);
    return pC_->writeController();
}

/** Performs a short epicsThreadSleep().
  *
  */
void Newport8743Axis::shortWait() {
    epicsThreadSleep(SHORTWAIT_SLEEP);
}

/** Checks if the motor is running in open- or closed-loop (i.e. its readback comes from the internal encoder)  for tiny/standard motor types.
  *
  * \return true if in closed-loop, false if open-loop or error
  */
bool Newport8743Axis::isClosedLoop() {
    bool closed = false;
    asynStatus status = asynError;

    if ((this->motorType == TINY_MOTOR) || (this->motorType == STANDARD_MOTOR)) {
        buildGenericGetCommand(pC_->outString_, AXIS_GETCLOSEDLOOP_CMD, this->axisNo_);
        status = pC_->writeReadController();
        if (status == asynSuccess) {
            closed = atoi(pC_->inString_);
        } else {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Error retrieving closed-loop status of Newport 8743-CL %s axis %d\n", pC_->portName, this->axisNo_);
        }
    }

    return closed;
}

/** Returns the current motion deadband.
  *
  * \return Current deadband, or -1 in case of error
  */
long Newport8743Axis::getDeadband() {
    asynStatus status = asynError;
    long deadband = -1;

    buildGenericGetCommand(pC_->outString_, AXIS_GETDEADBAND_CMD, this->axisNo_);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        deadband = atol(pC_->inString_);
    } else {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Error retrieving current deadband of Newport 8743-CL %s axis %d\n", pC_->portName, this->axisNo_);
    }

    return deadband;
}

/** Asks the controller for the type of motor attached to this axis.
  *
  * \return -1 for communication error, 0 no motor, 1 unknown type, 2 tiny motor, 3 standard motor
  */
newportMotorType Newport8743Axis::retrieveMotorType() {
    newportMotorType typ = COMM_FAILURE;

    buildGenericGetCommand(pC_->outString_, AXIS_MOTORTYPE_CMD, this->axisNo_);
    if (pC_->writeReadController() == asynSuccess) {
        typ = static_cast<newportMotorType>(atoi(pC_->inString_));
    }

    return typ;
}

/** All the following methods parse a reply sent by the controller.
  *
  */
bool Newport8743Axis::updateAxisReadbackPosition(asynStatus status, const char *reply, long& readback, asynStatus *asyn_error) {
    bool res = false;
    if ((status == asynSuccess) && (issigneddigit(reply))) {
        readback = atol(reply);
        res = true;
    } else {
        if (asyn_error) *asyn_error = asynError;
    }
    return res;
}

bool Newport8743Axis::updateAxisDeadband(asynStatus status, const char *reply, long& deadband, asynStatus *asyn_error) {
    bool res = false;
    if ((status == asynSuccess) && (issigneddigit(reply))) {
        deadband = atol(reply);
        res = true;
    } else {
        if (asyn_error) *asyn_error = asynError;
    }
    return res;
}

bool Newport8743Axis::updateAxisMotionDone(asynStatus status, const char *reply, bool& motion_done, asynStatus *asyn_error) {
    bool res = false;
    if ((status == asynSuccess) && (strlen(reply)) && ((*reply>='0') && (*reply<='1'))) {
        motion_done = atoi(reply);
        res = true;
    } else {
        if (asyn_error) *asyn_error = asynError;
    }
    return res;
}

bool Newport8743Axis::updateAxisClosedLoop(asynStatus status, const char *reply, bool& closed_loop, asynStatus *asyn_error) {
    bool res = false;
    if ((status == asynSuccess) && (strlen(reply)) && ((*reply>='0') && (*reply<='1'))) {
        closed_loop = atoi(reply);
        res = true;
    } else {
        if (asyn_error) *asyn_error = asynError;
    }
    return res;
}

bool Newport8743Axis::updateAxisMotorType(asynStatus status, const char *reply, newportMotorType& mot_type, asynStatus *asyn_error) {
    bool res = false;
    if ((status == asynSuccess) && (strlen(reply)==1) && ((*reply>='0') && (*reply<='3'))) {
        mot_type = static_cast<newportMotorType>(atoi(reply));
        res = true;
    } else {
        if (asyn_error) *asyn_error = asynError;
    }
    return res;
}

/** All the following methods generate a command string to be sent to the controller.
  *
  */
bool Newport8743Axis::buildMoveAbsoluteCommand(char *buffer, int axis, double abs_pos, double velocity, long deadband) {
    if ((!buffer) || (axis<0) || (axis>1)) {
        return false;
    }
    sprintf(buffer, AXIS_MOVEABS_CMD, axis+1, 1, axis+1, deadband, axis+1, (long)velocity, axis+1, (long)abs_pos);
    return true;
}

bool Newport8743Axis::buildMoveRelativeCommand(char *buffer, int axis, double rel_pos, double velocity, long deadband) {
    if ((!buffer) || (axis<0) || (axis>1)) {
        return false;
    }
    sprintf(buffer, AXIS_MOVEREL_CMD, axis+1, 1, axis+1, deadband, axis+1, (long)velocity, axis+1, (long)rel_pos);
    return true;
}

bool Newport8743Axis::buildHomeCommand(char *buffer, int axis, int forwards, newportHomeType home_type) {
    if ((!buffer) || (axis<0) || (axis>1)) {
        return false;
    }
    sprintf(buffer, HOME_COMMAND[home_type], axis+1, HOMING_DIR[forwards]);
    return true;
}

bool Newport8743Axis::buildStopCommand(char *buffer, int axis) {
    if ((!buffer) || (axis<0) || (axis>1)) {
        return false;
    }
    sprintf(buffer, AXIS_STOP_CMD, axis+1);
    return true;
}

bool Newport8743Axis::buildZeroCommand(char *buffer, int axis) {
    if ((!buffer) || (axis<0) || (axis>1)) {
        return false;
    }
    sprintf(buffer, AXIS_ZERO_CMD, axis+1);
    return true;
}

bool Newport8743Axis::buildSetPositionCommand(char *buffer, int axis, double position, long current_readback) {
    if ((!buffer) || (axis<0) || (axis>1)) {
        return false;
    }
    sprintf(buffer, AXIS_FORCEPOS_CMD, axis+1, (long)position);
    return true;
}

bool Newport8743Axis::buildCloseLoopCommand(char *buffer, int axis) {
    if ((!buffer) || (axis<0) || (axis>1)) {
        return false;
    }
    sprintf(buffer, AXIS_SETCLOSEDLOOP_CMD, axis+1, 1);
    return true;
}

bool Newport8743Axis::buildGenericGetCommand(char *buffer, const char *command_format, int axis) {
    if ((!buffer) || (!command_format) || (axis<0) || (axis>1)) {
        return false;
    }
    sprintf(buffer, command_format, axis+1);
    return true;
}

/** Called by the controller object after successfully polling the device,
  * either for the first time of after getting timeout errors.
  *
  * Beware: this controller regularly disconnects itself, so make sure this function
  * doesn't abort any ongoing motion!
  */
void Newport8743Axis::gotConnected() {
    int motion_done=0;
    newportMotorType motor_type = retrieveMotorType();

    if ((motor_type == TINY_MOTOR) || (motor_type == STANDARD_MOTOR)) {
        if (this->motorType == NO_MOTOR) { // Most likely first time connected
            setIntegerParam(pC_->motorStatusHomed_, 0);
            setIntegerParam(pC_->motorStatusHasEncoder_, 1);

            buildGenericGetCommand(pC_->outString_, AXIS_MOTIONDONE_CMD, this->axisNo_);
            if (pC_->writeReadController() == asynSuccess) {
                motion_done = atoi(pC_->inString_);
                if (motion_done) {
                    buildCloseLoopCommand(pC_->outString_, this->axisNo_);
                    if (pC_->writeController() == asynSuccess) {
                        setIntegerParam(pC_->motorClosedLoop_, 1);
                    }
                }
            }
        }

        this->motorType = motor_type;
        setStatusProblem(asynSuccess);
    } else {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Unsupported Newport 8743-CL %s axis %d type %d (must be tiny or standard)\n", pC_->portName, this->axisNo_, this->motorType);
        setStatusProblem(asynError);
    }
}

/** Called by the controller object if it gets a timeout error when polling the device.
  *
  */
void Newport8743Axis::gotDisconnected() {
    disconnectedFlag = true;
    this->motorType = COMM_FAILURE;
    setStatusProblem(asynTimeout);
}



/** Creates a new Newport8743Controller object.
  * Configuration command, called directly or from iocsh.
  *
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] asynPortName      The name of the drvAsynIPPort/drvAsynSerialPortConfigure that was created previously to connect to the Nanomotion controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  *
  * \return Always asynSuccess
  */
extern "C" int Newport8743CreateController(const char *portName, const char *asynPortName, int numAxes,  int movingPollPeriod, int idlePollPeriod) {
    new Newport8743Controller(portName, asynPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
    return asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg Newport8743CreateControllerArg0 = { "Port name", iocshArgString };
static const iocshArg Newport8743CreateControllerArg1 = { "Asyn port name", iocshArgString };
static const iocshArg Newport8743CreateControllerArg2 = { "Number of axes", iocshArgInt };
static const iocshArg Newport8743CreateControllerArg3 = { "Moving poll period (ms)", iocshArgInt };
static const iocshArg Newport8743CreateControllerArg4 = { "Idle poll period (ms)", iocshArgInt };
static const iocshArg * const Newport8743CCreateControllerArgs[] = { &Newport8743CreateControllerArg0,
                                                                     &Newport8743CreateControllerArg1,
                                                                     &Newport8743CreateControllerArg2,
                                                                     &Newport8743CreateControllerArg3,
                                                                     &Newport8743CreateControllerArg4 };
static const iocshFuncDef Newport8743CreateControllerDef = { "Newport8743CreateController", 5, Newport8743CCreateControllerArgs };
static void Newport8743CreateControllerCallFunc(const iocshArgBuf *args) {
    Newport8743CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void NP8743CLControllerRegister(void) {
    iocshRegister(&Newport8743CreateControllerDef, Newport8743CreateControllerCallFunc);
}

extern "C" {
    epicsExportRegistrar(NP8743CLControllerRegister);
}

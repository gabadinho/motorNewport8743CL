/*
FILENAME...   Newport8743MotorDriver.h
USAGE...      Motor driver support (model 3, asyn) for the Newport 8743-CL controller

Jose G.C. Gabadinho
September 2021
*/


#ifndef _NEWPORT8743MOTORDRIVER_H_
#define _NEWPORT8743MOTORDRIVER_H_

#include <asynMotorController.h>
#include <asynMotorAxis.h>



#define AXIS_RDBD_PARAMNAME     "MOTOR_RDBD"
#define AXIS_MRES_PARAMNAME     "MOTOR_MRES"
#define AXIS_HOMRTYPE_PARAMNAME "MOTOR_HOMR"
#define AXIS_HOMFTYPE_PARAMNAME "MOTOR_HOMF"
#define CTRL_RST_PARAMNAME      "CTRL_RESET"



const char CTRL_ID_CMD[]      = "*ID?";
const char CTRL_VERSION_CMD[] = "VE?";
const char CTRL_IPADDR_CMD[]  = "IPADDR?";

const char CTRL_RESET_CMD[] = "*RST";

const char CTRL_STATUS_CMD[]       = "PH?";
const char CTRL_ERRORCODE_CMD[]    = "TE?";
const char CTRL_ERRORMESSAGE_CMD[] = "TB?";

const char AXIS_MOTORTYPE_CMD[]   = "%dQM?";
const char AXIS_HARDWARECFG_CMD[] = "%dZH?";

const char AXIS_READBACKPOS_CMD[] = "%dTP?";
const char AXIS_MOTIONDONE_CMD[]  = "%dMD?";

const char AXIS_MOVEABS_CMD[]  = "%dMM%d ; %dDB%ld ; %dVA%ld ; %dPA%ld";
const char AXIS_MOVEREL_CMD[]  = "%dMM%d ; %dDB%ld ; %dVA%ld ; %dPR%ld";

const char AXIS_STOP_CMD[] = "%dST ; AB";

const char AXIS_ZERO_CMD[]     = "%dDH";
const char AXIS_FORCEPOS_CMD[] = "%dDH%ld";

const char AXIS_GETCLOSEDLOOP_CMD[] = "%dMM?";
const char AXIS_SETCLOSEDLOOP_CMD[] = "%dMM%d";

const char AXIS_GETDEADBAND_CMD[]  = "%dDB?";



const char HOMING_DIR[] = { '-', '+' };

const int AXIS_POS_LS     = 0x01;
const int AXIS_NEG_LS     = 0x02;
const int AXIS_HOMIDX_SIG = 0x04;

const int TINY_MAX_VEL = 1750;



enum newportMotorType {
    COMM_FAILURE = -1,
    NO_MOTOR,
    UNKNOWN_TYPE,
    TINY_MOTOR,
    STANDARD_MOTOR
};

enum newportHomeType {
    DISABLE = 0,
    FIND_HOME,
    FIND_INDEX,
    FIND_LIMIT
};
const char* HOME_COMMAND[] = {
    "",
    "%dOR",
    "%dMZ%c",
    "%dMT%c"
};



class Newport8743Axis: public asynMotorAxis {

public:
    Newport8743Axis(class Newport8743Controller *pC, int axis);

    // These are the methods we override from the base class
    void report(FILE *fp, int level);

    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus setPosition(double position);

    asynStatus poll(bool *moving);

    // Class-wide methods
    static bool updateAxisReadbackPosition(asynStatus status, const char *reply, long& readback, asynStatus *asyn_error);
    static bool updateAxisDeadband(asynStatus status, const char *reply, long& deadband, asynStatus *asyn_error);
    static bool updateAxisMotionDone(asynStatus status, const char *reply, bool& motion_done, asynStatus *asyn_error);
    static bool updateAxisClosedLoop(asynStatus status, const char *reply, bool& closed_loop, asynStatus *asyn_error);
    static bool updateAxisMotorType(asynStatus status, const char *reply, newportMotorType& mot_type, asynStatus *asyn_error);

    static bool buildMoveAbsoluteCommand(char *buffer, int axis, double abs_pos, double velocity, long deadband);
    static bool buildMoveRelativeCommand(char *buffer, int axis, double rel_pos, double velocity, long deadband);
    static bool buildHomeCommand(char *buffer, int axis, int forwards, newportHomeType home_type);
    static bool buildStopCommand(char *buffer, int axis);
    static bool buildZeroCommand(char *buffer, int axis);
    static bool buildSetPositionCommand(char *buffer, int axis, double position, long current_readback);
    static bool buildCloseLoopCommand(char *buffer, int axis);
    static bool buildGenericGetCommand(char *buffer, const char *command_format, int axis);

    static bool issigneddigit(const char *buffer) {
        size_t buf_len = strlen(buffer);
        if (buf_len == 1) return isdigit(*buffer);
        if (buf_len > 1) return isdigit(*buffer) || (*buffer=='-' && isdigit(*++buffer));
        return false;
    }

protected:
    // Specific class methods
    void setStatusProblem(asynStatus status);
    void updateAxisStatus(int axis_status);

    asynStatus stopMotor();

    void shortWait();

    bool isClosedLoop();

    long getDeadband();

    newportMotorType retrieveMotorType();

    void gotConnected();
    void gotDisconnected();

private:
    Newport8743Controller *pC_; // Pointer to the asynMotorController to which this axis belongs

    bool motionDone;
    int axisStatus;
    long positionReadback;
    newportMotorType motorType;

    bool disconnectedFlag;

friend class Newport8743Controller;
};

class Newport8743Controller: public asynMotorController {

public:
    Newport8743Controller(const char *portName, const char *asynPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

    // These are the methods we override from the base class
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    void report(FILE *fp, int level);

    Newport8743Axis* getAxis(asynUser *pasynUser);
    Newport8743Axis* getAxis(int axisNo);

    asynStatus poll();

    // Static class methods
    static bool updateControllerStatus(asynStatus status, const char *reply, int& ctrl_stat, asynStatus *asyn_error);

    static bool buildGenericGetCommand(char *buffer, const char *command_format);

protected:
    // Specific class methods
    void gotConnected();
    asynStatus gotDisconnected();

    int driverRetryDeadband;
    int driverMotorRecResolution;
    int driverHomeReverseType;
    int driverHomeForwardType;
    int driverResetController;
#define NUM_NEWPORT8743_PARAMS 5

private:
    bool disconnectedFlag;

friend class Newport8743Axis;
};

#endif // _NEWPORT8743MOTORDRIVER_H_

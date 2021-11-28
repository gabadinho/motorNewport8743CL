#include <gtest/gtest.h>

#include "Newport8743MotorDriver.h"



TEST(ReplyParse, AsynErrorStatus) {
    char reply[] = "1000";
    long rb;
    asynStatus asyn_error = asynSuccess;
    bool res = Newport8743Axis::updateAxisReadbackPosition(asynError, reply, rb, &asyn_error);
    ASSERT_EQ(false, res);
    ASSERT_EQ(asynError, asyn_error);
}

TEST(ReplyParse, ZeroReadback) {
    char reply[] = "0";
    long rb;
    asynStatus asyn_error = asynSuccess;
    bool res = Newport8743Axis::updateAxisReadbackPosition(asynSuccess, reply, rb, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(0, rb);
    ASSERT_EQ(asynSuccess, asyn_error);
}

TEST(ReplyParse, PositiveReadback) {
    char reply[] = "1000";
    long rb;
    asynStatus asyn_error = asynSuccess;
    bool res = Newport8743Axis::updateAxisReadbackPosition(asynSuccess, reply, rb, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(1000, rb);
    ASSERT_EQ(asynSuccess, asyn_error);
}

TEST(ReplyParse, NegativeReadback) {
    char reply[] = "-2500";
    long rb;
    asynStatus asyn_error = asynSuccess;
    bool res = Newport8743Axis::updateAxisReadbackPosition(asynSuccess, reply, rb, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(-2500, rb);
    ASSERT_EQ(asynSuccess, asyn_error);
}



TEST(ReplyParse, NullDeadband) {
    char reply[] = "";
    long rb;
    asynStatus asyn_error = asynSuccess;
    bool res = Newport8743Axis::updateAxisDeadband(asynSuccess, reply, rb, &asyn_error);
    ASSERT_EQ(false, res);
    ASSERT_EQ(asynError, asyn_error);
}

TEST(ReplyParse, GetDeadband) {
    char reply[] = "10000";
    long rb;
    asynStatus asyn_error = asynSuccess;
    bool res = Newport8743Axis::updateAxisDeadband(asynSuccess, reply, rb, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(10000, rb);
    ASSERT_EQ(asynSuccess, asyn_error);
}



TEST(ReplyParse, MotionDone) {
    char reply[] = "1";
    bool md;
    asynStatus asyn_error = asynSuccess;
    bool res = Newport8743Axis::updateAxisMotionDone(asynSuccess, reply, md, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(true, md);
    ASSERT_EQ(asynSuccess, asyn_error);
}

TEST(ReplyParse, MotionNotDone) {
    char reply[] = "0";
    bool md;
    asynStatus asyn_error = asynSuccess;
    bool res = Newport8743Axis::updateAxisMotionDone(asynSuccess, reply, md, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(false, md);
    ASSERT_EQ(asynSuccess, asyn_error);
}



TEST(ReplyParse, CloseLoop) {
    char reply[] = "1";
    bool cl;
    asynStatus asyn_error = asynSuccess;
    bool res = Newport8743Axis::updateAxisClosedLoop(asynSuccess, reply, cl, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(true, cl);
    ASSERT_EQ(asynSuccess, asyn_error);
}

TEST(ReplyParse, OpenLoop) {
    char reply[] = "0";
    bool cl;
    asynStatus asyn_error = asynSuccess;
    bool res = Newport8743Axis::updateAxisClosedLoop(asynSuccess, reply, cl, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(false, cl);
    ASSERT_EQ(asynSuccess, asyn_error);
}



TEST(ReplyParse, MotorTypeTiny) {
    char reply[] = "2";
    newportMotorType mt;
    asynStatus asyn_error = asynSuccess;
    bool res = Newport8743Axis::updateAxisMotorType(asynSuccess, reply, mt, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(TINY_MOTOR, mt);
    ASSERT_EQ(asynSuccess, asyn_error);
}

TEST(ReplyParse, MotorTypeStandard) {
    char reply[] = "3";
    newportMotorType mt;
    asynStatus asyn_error = asynSuccess;
    bool res = Newport8743Axis::updateAxisMotorType(asynSuccess, reply, mt, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(STANDARD_MOTOR, mt);
    ASSERT_EQ(asynSuccess, asyn_error);
}

TEST(ReplyParse, NoMotor) {
    char reply[] = "0";
    newportMotorType mt;
    asynStatus asyn_error = asynSuccess;
    bool res = Newport8743Axis::updateAxisMotorType(asynSuccess, reply, mt, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(NO_MOTOR, mt);
    ASSERT_EQ(asynSuccess, asyn_error);
}


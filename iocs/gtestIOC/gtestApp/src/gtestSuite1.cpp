#include <gtest/gtest.h>

#include "Newport8743MotorDriver.h"



#define STRING_BUFFER_SIZE 256



TEST(CommandBuild, MoveAbs_0_1000000_50000_20000) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildMoveAbsoluteCommand(buffer, 0, 1000000, 50000, 20000);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("1MM1 ; 1DB20000 ; 1VA50000 ; 1PA1000000", buffer);
}

TEST(CommandBuild, MoveAbs_0_0_50000_20000) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildMoveAbsoluteCommand(buffer, 0, 0, 50000, 20000);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("1MM1 ; 1DB20000 ; 1VA50000 ; 1PA0", buffer);
}

TEST(CommandBuild, MoveAbs_1_min300000_40000_10000) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildMoveAbsoluteCommand(buffer, 1, -3000000, 40000, 10000);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("2MM1 ; 2DB10000 ; 2VA40000 ; 2PA-3000000", buffer);
}



TEST(CommandBuild, MoveRel_0_1000000_50000_20000) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildMoveRelativeCommand(buffer, 0, 1000000, 50000, 20000);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("1MM1 ; 1DB20000 ; 1VA50000 ; 1PR1000000", buffer);
}

TEST(CommandBuild, MoveRel_1_min300000_40000_10000) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildMoveRelativeCommand(buffer, 1, -3000000, 40000, 10000);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("2MM1 ; 2DB10000 ; 2VA40000 ; 2PR-3000000", buffer);
}



TEST(CommandBuild, Home_0_For_Disable) {
    char buffer[STRING_BUFFER_SIZE] = "";
    bool res = Newport8743Axis::buildHomeCommand(buffer, 0, 1, DISABLE);
    ASSERT_EQ(false, res);
    ASSERT_STREQ("", buffer);
}

TEST(CommandBuild, Home_0_For_FindHome) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildHomeCommand(buffer, 0, 1, FIND_HOME);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("1OR", buffer);
}

TEST(CommandBuild, Home_0_For_FindLimit) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildHomeCommand(buffer, 0, 1, FIND_LIMIT);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("1MT+", buffer);
}

TEST(CommandBuild, Home_1_Rev_FindIndex) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildHomeCommand(buffer, 1, 0, FIND_INDEX);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("2MZ-", buffer);
}



TEST(CommandBuild, Stop_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildStopCommand(buffer, 0);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("1ST ; AB", buffer);
}



TEST(CommandBuild, Zero_1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildZeroCommand(buffer, 1);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("2DH", buffer);
}



TEST(CommandBuild, SetPos_0_125000) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildSetPositionCommand(buffer, 0, 125000);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("1DH125000", buffer);
}

TEST(CommandBuild, SetPos_1_min5000) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildSetPositionCommand(buffer, 1, -5000);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("2DH-5000", buffer);
}


TEST(CommandBuild, CloseLoop_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildCloseLoopCommand(buffer, 0);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("1MM1", buffer);
}



TEST(CommandBuild, GenCmdReadback_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = Newport8743Axis::buildGenericGetCommand(buffer, AXIS_READBACKPOS_CMD, 0);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("1TP?", buffer);
}


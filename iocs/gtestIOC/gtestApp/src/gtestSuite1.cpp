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


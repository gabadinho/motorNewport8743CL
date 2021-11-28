#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "Newport8743MotorDriver.h"



class MockNewport8743Controller: public Newport8743Controller {
public:
    MockNewport8743Controller(const char *portName, const char *asynPortName, int numAxes, double movingPollPeriod, double idlePollPeriod):
        Newport8743Controller(portName, asynPortName, numAxes, movingPollPeriod, idlePollPeriod) {
        shuttingDown_ = 1;
    }

    void log(int reason, const char *format, ...) {}

    MOCK_METHOD(asynStatus, connect, (asynUser*), (override));
    MOCK_METHOD(asynStatus, disconnect, (asynUser*), (override));

    void gotConnected() {
        return Newport8743Controller::gotConnected();
    }
    asynStatus gotDisconnected() {
        return Newport8743Controller::gotDisconnected();
    }
};

class MockNewport8743Axis: public Newport8743Axis {
public:
    MockNewport8743Axis(MockNewport8743Controller *ctrl, newportMotorType motor_type, int axis): Newport8743Axis(ctrl, axis) {
        this->motorType = motor_type;
    }

    void log(int reason, const char *format, ...) {}

    MOCK_METHOD(void, gotConnected, (), (override));
    MOCK_METHOD(void, gotDisconnected, (), (override));

    MOCK_METHOD(newportMotorType, retrieveMotorType, (), (override));
};



TEST(ctrlConnection, GotConnected) {
    MockNewport8743Controller dummy_ctrl("fake_ctrl", "fake_conn", 2, 1.0, 1.0);
    MockNewport8743Axis dummy_axis(&dummy_ctrl, TINY_MOTOR, 0);
    ON_CALL(dummy_axis, retrieveMotorType()).WillByDefault(testing::Return(TINY_MOTOR));
    EXPECT_CALL(dummy_axis, gotConnected());
    dummy_ctrl.gotConnected();
}

TEST(ctrlConnection, GotDisconnected) {
    MockNewport8743Controller dummy_ctrl("fake_ctrl2", "fake_conn2", 2, 1.0, 1.0);
    MockNewport8743Axis dummy_axis(&dummy_ctrl, TINY_MOTOR, 1);
    EXPECT_CALL(dummy_axis, gotDisconnected());
    EXPECT_CALL(dummy_ctrl, connect(testing::_));
    EXPECT_CALL(dummy_ctrl, disconnect(testing::_));
    dummy_ctrl.gotDisconnected();
}


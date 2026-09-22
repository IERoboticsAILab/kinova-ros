#include <gtest/gtest.h>
#include <kinova_driver/kinova_control_mode.h>

#include <stdexcept>
#include <string>

namespace
{
class ControlModeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        active = this;
        api.getTrajectoryTorqueMode = readMode;
        api.switchTrajectoryTorque = switchMode;
    }
    static int readMode(int &mode)
    {
        active->calls += 'R';
        mode = active->modes[active->reads];
        return active->read_results[active->reads++];
    }
    static int switchMode(GENERALCONTROL_TYPE target)
    {
        active->calls += 'W';
        active->requested = target;
        return active->switch_result;
    }
    void ensure(GENERALCONTROL_TYPE target = POSITION)
    {
        kinova::ensureTrajectoryControlMode(api, target);
    }
    static ControlModeTest *active;
    kinova::KinovaAPI api;
    int modes[3] = {POSITION, POSITION, POSITION};
    int read_results[3] = {NO_ERROR_KINOVA, NO_ERROR_KINOVA, NO_ERROR_KINOVA};
    int switch_result = NO_ERROR_KINOVA;
    int reads = 0;
    GENERALCONTROL_TYPE requested = TORQUE;
    std::string calls;
};
ControlModeTest *ControlModeTest::active = nullptr;

TEST_F(ControlModeTest, AlreadyPositionSkipsSwitch)
{
    EXPECT_NO_THROW(ensure());
    EXPECT_EQ("R", calls);
}

TEST_F(ControlModeTest, AlreadyTorqueSkipsSwitch)
{
    modes[0] = TORQUE;
    EXPECT_NO_THROW(ensure(TORQUE));
    EXPECT_EQ("R", calls);
}

TEST_F(ControlModeTest, TorqueToPositionVerifiesReadback)
{
    modes[0] = TORQUE;
    EXPECT_NO_THROW(ensure());
    EXPECT_EQ("RWR", calls);
    EXPECT_EQ(POSITION, requested);
}

TEST_F(ControlModeTest, PositionToTorqueVerifiesReadback)
{
    modes[1] = TORQUE;
    EXPECT_NO_THROW(ensure(TORQUE));
    EXPECT_EQ("RWR", calls);
    EXPECT_EQ(TORQUE, requested);
}

TEST_F(ControlModeTest, FailedReadDoesNotSwitchOrTrustItsOutput)
{
    for (int reported : {POSITION, TORQUE})
    {
        reads = 0;
        calls.clear();
        modes[0] = reported;
        read_results[0] = JACO_COMM_FAILED;
        EXPECT_THROW(ensure(), kinova::KinovaCommException);
        EXPECT_EQ("R", calls);
    }
}

TEST_F(ControlModeTest, UnknownModeDoesNotSwitch)
{
    modes[0] = 99;
    EXPECT_THROW(ensure(), std::runtime_error);
    EXPECT_EQ("R", calls);
}

TEST_F(ControlModeTest, FailedSwitchPreservesCommunicationError)
{
    modes[0] = TORQUE;
    switch_result = JACO_COMM_FAILED;
    try
    {
        ensure();
        FAIL() << "Expected the switch failure to propagate";
    }
    catch (const kinova::KinovaCommException &error)
    {
        EXPECT_NE(std::string::npos, std::string(error.what()).find("2004"));
    }
    EXPECT_EQ("RW", calls);
}

TEST_F(ControlModeTest, FailedReadbackDoesNotReportSuccess)
{
    modes[0] = TORQUE;
    read_results[1] = JACO_COMM_FAILED;
    EXPECT_THROW(ensure(), kinova::KinovaCommException);
    EXPECT_EQ("RWR", calls);
}

TEST_F(ControlModeTest, MismatchedReadbackDoesNotReportSuccess)
{
    modes[0] = modes[1] = TORQUE;
    EXPECT_THROW(ensure(), std::runtime_error);
    EXPECT_EQ("RWR", calls);
}

TEST_F(ControlModeTest, RetryReadsFreshStateAfterFailedSwitch)
{
    modes[0] = TORQUE;
    switch_result = JACO_COMM_FAILED;
    EXPECT_THROW(ensure(), kinova::KinovaCommException);
    EXPECT_NO_THROW(ensure());
    EXPECT_EQ("RWR", calls);
}

TEST_F(ControlModeTest, MissingGetterFailsWithoutSwitch)
{
    api.getTrajectoryTorqueMode = nullptr;
    EXPECT_THROW(ensure(), std::runtime_error);
    EXPECT_TRUE(calls.empty());
}

TEST_F(ControlModeTest, MissingSetterOnlyFailsWhenSwitchNeeded)
{
    api.switchTrajectoryTorque = nullptr;
    EXPECT_NO_THROW(ensure());
    modes[1] = TORQUE;
    EXPECT_THROW(ensure(), std::runtime_error);
    EXPECT_EQ("RR", calls);
}
}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::Time::init();  // ROS logging only; no node, master, or hardware connection.
    return RUN_ALL_TESTS();
}

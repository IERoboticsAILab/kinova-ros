#include <gtest/gtest.h>
#include <kinova_driver/kinova_velocity_diagnostics.h>

#include <string>

namespace
{
class VelocityDiagnosticsTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        active = this;
        api.getQuickStatus = readQuick;
        api.getGlobalTrajectoryInfo = readFifo;
        api.getControlType = readControl;
        api.getTrajectoryTorqueMode = readTorque;

        // Guard the operations that must never be used to collect diagnostics.
        api.initAPI = unexpectedMutation;
        api.closeAPI = unexpectedMutation;
        api.startControlAPI = unexpectedMutation;
        api.stopControlAPI = unexpectedMutation;
        api.startForceControl = unexpectedMutation;
        api.stopForceControl = unexpectedMutation;
        api.setAngularControl = unexpectedMutation;
        api.setCartesianControl = unexpectedMutation;
        api.eraseAllTrajectories = unexpectedMutation;
        api.initFingers = unexpectedMutation;
        api.moveHome = unexpectedMutation;
        api.switchTrajectoryTorque = unexpectedModeSwitch;
        api.sendBasicTrajectory = unexpectedTrajectory;
        api.sendAdvanceTrajectory = unexpectedTrajectory;
    }

    static int readQuick(QuickStatus& value)
    {
        active->calls += 'Q';
        value.ControlEnableStatus = 255;
        value.ControlActiveModule = 42;
        value.ControlFrameType = 3;
        value.CartesianFaultState = 254;
        value.ForceControlStatus = 8;
        value.CurrentLimitationStatus = 9;
        value.RetractType = 10;
        value.Finger1Status = 11;
        value.Finger2Status = 12;
        value.Finger3Status = 13;
        return active->quick_result;
    }
    static int readFifo(TrajectoryFIFO& value)
    {
        active->calls += 'F';
        value.TrajectoryCount = 17;
        value.UsedPercentage = 12.5f;
        value.MaxSize = 2000;
        return active->fifo_result;
    }
    static int readControl(int& value)
    {
        active->calls += 'C';
        value = 23;
        return active->control_result;
    }
    static int readTorque(int& value)
    {
        active->calls += 'T';
        value = 24;
        return active->torque_result;
    }
    static int unexpectedMutation()
    {
        ++active->mutations;
        return NO_ERROR_KINOVA;
    }
    static int unexpectedModeSwitch(GENERALCONTROL_TYPE)
    {
        return unexpectedMutation();
    }
    static int unexpectedTrajectory(TrajectoryPoint)
    {
        return unexpectedMutation();
    }

    static VelocityDiagnosticsTest* active;
    kinova::KinovaAPI api;
    int quick_result = NO_ERROR_KINOVA;
    int fifo_result = NO_ERROR_KINOVA;
    int control_result = NO_ERROR_KINOVA;
    int torque_result = NO_ERROR_KINOVA;
    int mutations = 0;
    std::string calls;
};
VelocityDiagnosticsTest* VelocityDiagnosticsTest::active = nullptr;

TEST(VelocityDiagnosticStatus, DefaultsAreUnavailableAndZero)
{
    const kinova::VelocityDiagnosticStatus status;
    EXPECT_EQ(-1, status.quick_result);
    EXPECT_EQ(-1, status.fifo_result);
    EXPECT_EQ(-1, status.control_result);
    EXPECT_EQ(-1, status.torque_result);
    EXPECT_EQ(-1, status.control_type);
    EXPECT_EQ(-1, status.torque_mode);
    EXPECT_EQ(0, status.quick.ControlEnableStatus);
    EXPECT_EQ(0, status.quick.CartesianFaultState);
    EXPECT_EQ(0, status.quick.Finger1Status);
    EXPECT_EQ(0u, status.fifo.TrajectoryCount);
    EXPECT_FLOAT_EQ(0.0f, status.fifo.UsedPercentage);
    EXPECT_EQ(0u, status.fifo.MaxSize);
    EXPECT_EQ("quick_rc=-1 fifo_rc=-1 control_rc=-1 torque_rc=-1",
              kinova::formatVelocityDiagnosticStatus(status));
}

TEST_F(VelocityDiagnosticsTest, CallsEachGetterOnceWithoutMutatingRobot)
{
    const auto status = kinova::readVelocityDiagnosticStatus(api);
    EXPECT_EQ("QFCT", calls);
    EXPECT_EQ(0, mutations);
    EXPECT_EQ(NO_ERROR_KINOVA, status.quick_result);
    EXPECT_EQ(NO_ERROR_KINOVA, status.fifo_result);
    EXPECT_EQ(NO_ERROR_KINOVA, status.control_result);
    EXPECT_EQ(NO_ERROR_KINOVA, status.torque_result);
}

TEST_F(VelocityDiagnosticsTest, MissingGettersRemainUnavailable)
{
    api.getQuickStatus = nullptr;
    api.getGlobalTrajectoryInfo = nullptr;
    api.getControlType = nullptr;
    api.getTrajectoryTorqueMode = nullptr;
    const auto status = kinova::readVelocityDiagnosticStatus(api);
    EXPECT_TRUE(calls.empty());
    EXPECT_EQ(0, mutations);
    EXPECT_EQ("quick_rc=-1 fifo_rc=-1 control_rc=-1 torque_rc=-1",
              kinova::formatVelocityDiagnosticStatus(status));
}

TEST_F(VelocityDiagnosticsTest, OneMissingGetterDoesNotPreventOtherReads)
{
    api.getQuickStatus = nullptr;
    const auto status = kinova::readVelocityDiagnosticStatus(api);
    EXPECT_EQ("FCT", calls);
    EXPECT_EQ(0, mutations);
    EXPECT_EQ(-1, status.quick_result);
    const std::string text = kinova::formatVelocityDiagnosticStatus(status);
    EXPECT_EQ(std::string::npos, text.find("raw_control_enable="));
    EXPECT_NE(std::string::npos, text.find("fifo_count=17"));
    EXPECT_NE(std::string::npos, text.find("raw_control_type=23"));
    EXPECT_NE(std::string::npos, text.find("raw_trajectory_torque_mode=24"));
}

TEST_F(VelocityDiagnosticsTest, FailedReadsKeepCodesAndSuppressUntrustedFields)
{
    quick_result = JACO_COMM_FAILED;
    fifo_result = 2005;
    control_result = 0;
    torque_result = -99;
    const auto status = kinova::readVelocityDiagnosticStatus(api);
    EXPECT_EQ("QFCT", calls);
    EXPECT_EQ(0, mutations);
    EXPECT_EQ(JACO_COMM_FAILED, status.quick_result);
    EXPECT_EQ(2005, status.fifo_result);
    EXPECT_EQ(0, status.control_result);
    EXPECT_EQ(-99, status.torque_result);
    // The fakes wrote plausible data despite failing; none may be reported.
    EXPECT_EQ("quick_rc=2004 fifo_rc=2005 control_rc=0 torque_rc=-99",
              kinova::formatVelocityDiagnosticStatus(status));
}

TEST_F(VelocityDiagnosticsTest, PartialFailureStillReportsSuccessfulReads)
{
    fifo_result = JACO_COMM_FAILED;
    const auto status = kinova::readVelocityDiagnosticStatus(api);
    const std::string text = kinova::formatVelocityDiagnosticStatus(status);
    EXPECT_EQ("QFCT", calls);
    EXPECT_EQ(0, mutations);
    EXPECT_NE(std::string::npos, text.find("fifo_rc=2004"));
    EXPECT_EQ(std::string::npos, text.find("fifo_count="));
    EXPECT_EQ(std::string::npos, text.find("fifo_used_percent="));
    EXPECT_EQ(std::string::npos, text.find("fifo_max="));
    EXPECT_NE(std::string::npos, text.find("raw_control_enable=255"));
    EXPECT_NE(std::string::npos, text.find("raw_control_type=23"));
    EXPECT_NE(std::string::npos, text.find("raw_trajectory_torque_mode=24"));
}

TEST_F(VelocityDiagnosticsTest, FormatsRawFlagsAsNumbersWithoutEnumInterpretation)
{
    const auto status = kinova::readVelocityDiagnosticStatus(api);
    const std::string text = kinova::formatVelocityDiagnosticStatus(status);
    EXPECT_NE(std::string::npos, text.find("raw_control_enable=255"));
    EXPECT_NE(std::string::npos, text.find("raw_module=42"));
    EXPECT_NE(std::string::npos, text.find("raw_frame=3"));
    EXPECT_NE(std::string::npos, text.find("raw_cartesian_fault=254"));
    EXPECT_NE(std::string::npos, text.find("raw_force=8"));
    EXPECT_NE(std::string::npos, text.find("raw_current_limit=9"));
    EXPECT_NE(std::string::npos, text.find("raw_retract=10"));
    EXPECT_NE(std::string::npos, text.find("raw_finger1_init=11"));
    EXPECT_NE(std::string::npos, text.find("raw_finger2_init=12"));
    EXPECT_NE(std::string::npos, text.find("raw_finger3_init=13"));
    EXPECT_NE(std::string::npos, text.find("fifo_count=17"));
    EXPECT_NE(std::string::npos, text.find("fifo_used_percent=12.5"));
    EXPECT_NE(std::string::npos, text.find("fifo_max=2000"));
    EXPECT_NE(std::string::npos, text.find("raw_control_type=23"));
    EXPECT_NE(std::string::npos, text.find("raw_trajectory_torque_mode=24"));
}
}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

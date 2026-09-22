#ifndef KINOVA_DRIVER_KINOVA_VELOCITY_DIAGNOSTICS_H
#define KINOVA_DRIVER_KINOVA_VELOCITY_DIAGNOSTICS_H

#include <sstream>
#include <string>

#include "kinova_driver/kinova_api.h"

namespace kinova
{

struct VelocityDiagnosticStatus
{
    VelocityDiagnosticStatus()
        : quick(), fifo(), control_type(-1), torque_mode(-1),
          quick_result(-1), fifo_result(-1), control_result(-1), torque_result(-1)
    {
    }

    QuickStatus quick;
    TrajectoryFIFO fifo;
    int control_type;
    int torque_mode;
    int quick_result;
    int fifo_result;
    int control_result;
    int torque_result;
};

// The caller must hold the driver's API mutex. These queries do not change
// control modes, clear trajectories, restart the API, or send motion commands.
inline VelocityDiagnosticStatus readVelocityDiagnosticStatus(KinovaAPI& api)
{
    VelocityDiagnosticStatus status;
    if (api.getQuickStatus != nullptr)
        status.quick_result = api.getQuickStatus(status.quick);
    if (api.getGlobalTrajectoryInfo != nullptr)
        status.fifo_result = api.getGlobalTrajectoryInfo(status.fifo);
    if (api.getControlType != nullptr)
        status.control_result = api.getControlType(status.control_type);
    if (api.getTrajectoryTorqueMode != nullptr)
        status.torque_result = api.getTrajectoryTorqueMode(status.torque_mode);
    return status;
}

inline std::string formatVelocityDiagnosticStatus(const VelocityDiagnosticStatus& status)
{
    std::ostringstream out;
    out << "quick_rc=" << status.quick_result
        << " fifo_rc=" << status.fifo_result
        << " control_rc=" << status.control_result
        << " torque_rc=" << status.torque_result;

    // Preserve numeric firmware values. Do not infer boolean or enum meanings
    // from SDK comments, which vary across firmware versions and fields.
    if (status.quick_result == NO_ERROR_KINOVA)
    {
        out << " raw_control_enable=" << static_cast<unsigned int>(status.quick.ControlEnableStatus)
            << " raw_module=" << static_cast<unsigned int>(status.quick.ControlActiveModule)
            << " raw_frame=" << static_cast<unsigned int>(status.quick.ControlFrameType)
            << " raw_cartesian_fault=" << static_cast<unsigned int>(status.quick.CartesianFaultState)
            << " raw_force=" << static_cast<unsigned int>(status.quick.ForceControlStatus)
            << " raw_current_limit=" << static_cast<unsigned int>(status.quick.CurrentLimitationStatus)
            << " raw_retract=" << static_cast<unsigned int>(status.quick.RetractType)
            << " raw_finger1_init=" << static_cast<unsigned int>(status.quick.Finger1Status)
            << " raw_finger2_init=" << static_cast<unsigned int>(status.quick.Finger2Status)
            << " raw_finger3_init=" << static_cast<unsigned int>(status.quick.Finger3Status);
    }
    if (status.fifo_result == NO_ERROR_KINOVA)
    {
        out << " fifo_count=" << status.fifo.TrajectoryCount
            << " fifo_used_percent=" << status.fifo.UsedPercentage
            << " fifo_max=" << status.fifo.MaxSize;
    }
    if (status.control_result == NO_ERROR_KINOVA)
        out << " raw_control_type=" << status.control_type;
    if (status.torque_result == NO_ERROR_KINOVA)
        out << " raw_trajectory_torque_mode=" << status.torque_mode;
    return out.str();
}

}  // namespace kinova

#endif

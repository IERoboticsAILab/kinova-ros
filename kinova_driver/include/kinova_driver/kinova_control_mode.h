#ifndef KINOVA_DRIVER_KINOVA_CONTROL_MODE_H
#define KINOVA_DRIVER_KINOVA_CONTROL_MODE_H

#include <stdexcept>
#include <string>

#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_ros_types.h"

namespace kinova
{

// The caller must hold the driver's API mutex across the read/switch/read.
inline int readTrajectoryControlMode(KinovaAPI& api)
{
    if (api.getTrajectoryTorqueMode == nullptr)
    {
        throw std::runtime_error("Kinova API does not provide GetTrajectoryTorqueMode");
    }

    int mode = -1;
    const int result = api.getTrajectoryTorqueMode(mode);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not read the trajectory/torque control mode", result);
    }
    if (mode != POSITION && mode != TORQUE)
    {
        throw std::runtime_error("Kinova API returned unknown trajectory/torque mode: "
                                 + std::to_string(mode));
    }
    return mode;
}

inline void ensureTrajectoryControlMode(KinovaAPI& api, GENERALCONTROL_TYPE target)
{
    // GetControlType reports Cartesian vs angular control, not position vs torque.
    if (readTrajectoryControlMode(api) == target)
    {
        ROS_DEBUG("Kinova is already in %s control", target == POSITION ? "position" : "torque");
        return;
    }
    if (api.switchTrajectoryTorque == nullptr)
    {
        throw std::runtime_error("Kinova API does not provide SwitchTrajectoryTorque");
    }

    ROS_INFO("Switching to %s control", target == POSITION ? "position" : "torque");
    const int result = api.switchTrajectoryTorque(target);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set the torque control state", result);
    }
    if (readTrajectoryControlMode(api) != target)
    {
        throw std::runtime_error("Kinova did not confirm the requested control mode");
    }
}

}  // namespace kinova

#endif

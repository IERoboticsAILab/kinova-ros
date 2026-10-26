#ifndef KINOVA_JOINT_TRAJECTORY_CONTROLLER_H
#define KINOVA_JOINT_TRAJECTORY_CONTROLLER_H


#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Duration.h>
#include <kinova_msgs/JointVelocity.h>
#include <boost/thread.hpp>

#include "kinova_ros_types.h"
#include "kinova_comm.h"
#include "kinova_api.h"

namespace kinova
{

class JointTrajectoryController
{
public:
    JointTrajectoryController(kinova::KinovaComm &kinova_comm, ros::NodeHandle &n);
    ~JointTrajectoryController();


private:
    ros::NodeHandle nh_;

    ros::Subscriber sub_command_;
    ros::Publisher pub_joint_feedback_;
    ros::Publisher pub_joint_velocity_;

    ros::Time previous_pub_;
    ros::Time time_pub_joint_vel_;

    ros::Timer timer_pub_joint_vel_;
    boost::mutex terminate_thread_mutex_;
    boost::thread* thread_update_state_;
    bool terminate_thread_;

    std_msgs::Duration time_from_start_;
    sensor_msgs::JointState current_joint_state_;

    KinovaComm kinova_comm_;
    TrajectoryPoint kinova_traj_point_;

//    trajectory_msgs::JointTrajectory joint_traj_;
//    trajectory_msgs::JointTrajectoryPoint joint_traj_point_;
    std::string traj_frame_id_;
    std::vector<trajectory_msgs::JointTrajectoryPoint> traj_command_points_;
    control_msgs::FollowJointTrajectoryFeedback traj_feedback_msg_;

    // stores the command to send to robot, in Kinova type (KinovaAngles)
    std::vector<KinovaAngles> kinova_angle_command_;

    uint number_joint_;
    int traj_command_points_index_;
    std::vector<std::string> joint_names_;
    std::string prefix_;

    struct Segment
    {
        double start_time;
        double duration;
        std::vector<double> positions;
        std::vector<double> velocities;
    };


    // Closed-loop tracking (trajectory_controller/closed_loop). The open-loop
    // path sends each point's velocity for that point's time and never looks
    // at the joints, so a move can "finish" centimetres short. Closed loop
    // sends the trajectory's velocity at this instant plus a correction
    // towards its position at this instant, then holds the last point until
    // the joints are on it.
    bool closed_loop_;
    double position_gain_;          // 1/s
    double max_correction_;         // rad/s, per joint
    double max_tracking_error_;     // rad: further behind than this aborts
    double goal_tolerance_;         // rad: every joint within this is done
    double settle_timeout_;         // s after the trajectory's end
    bool tracking_;
    ros::Time tracking_start_;
    size_t tracking_segment_;
    double tracking_max_error_;
    std::vector<double> plan_times_;
    std::vector<std::vector<double> > plan_positions_;   // per point, rad, unwrapped
    std::vector<std::vector<double> > plan_velocities_;  // per point, rad/s

    // call back function when receive a trajectory command
    void commandCB(const trajectory_msgs::JointTrajectoryConstPtr &traj_msg);
    void startClosedLoop(const trajectory_msgs::JointTrajectory &traj, const std::vector<int> &lookup);
    void closedLoopStep();
    void stopClosedLoop();
    void sampleTrajectory(double t, std::vector<double> &q, std::vector<double> &v);

    // reflash the robot state and publish the joint state: either by timer or thread
    void update_state(); // by thread

    void pub_joint_vel(const ros::TimerEvent&); // by timer
    int test;

};






}


#endif // KINOVA_JOINT_TRAJECTORY_CONTROLLER_H

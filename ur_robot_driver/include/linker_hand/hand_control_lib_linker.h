/*********************************************************************************************//**
* hand_control_lib.h

* This file contains the class definition for the hand_comm class. This class is used to
* communicate with the Inspire Hand via serial communication.

* October 2024
* Author: Shuo Liu

*********************************************************************************************/

#ifndef HAND_CONTROL_LIB_LINKER_H
#define HAND_CONTROL_LIB_LINKER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <mutex>

namespace linker_hand {

class hand_comm {
public:
    // Constructors and Destructor
    hand_comm() = default;
    hand_comm(ros::NodeHandle* nh, const std::string& hand_prefix, const int& joint_nums);
    ~hand_comm() = default;

    // Set NodeHandle
    void set_nh(ros::NodeHandle* nh, const std::string& hand_prefix, const int& joint_nums);

    // Set the angle of the hand (limited by definition above)
    bool set_angle(const std::vector<double>& angles);

    // Callback function for receiving hand joint state
    void hand_joint_state_cb(const sensor_msgs::JointStateConstPtr& msg);

    std::vector<double> curangle_;
    std::vector<double> curspeed_;
    std::vector<double> curforce_;
    std::vector<double> setangle_;
    std::vector<double> setangle_cmd_;
    int joint_nums_;

    // Define the maximum and minimum angle limits for the hand
    std::vector<double> angle_upper_limit, angle_lower_limit, angle_encoder_upper_limit, angle_encoder_lower_limit;

    ros::Subscriber joint_state_sub_;
    ros::Publisher joint_state_pub_;
};

} // namespace linker_hand

#endif // HAND_CONTROL_LIB_LINKER_H

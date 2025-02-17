#ifndef HAND_CONTROL_LIB_LINKER_CPP
#define HAND_CONTROL_LIB_LINKER_CPP

#include <linker_hand/hand_control_lib_linker.h>

#include <vector>
#include <iostream>
#include <string>

namespace linker_hand {

hand_comm::hand_comm(ros::NodeHandle *nh, const std::string& hand_prefix, const int& joint_nums) {
    set_nh(nh, hand_prefix, joint_nums);
}

void hand_comm::set_nh(ros::NodeHandle* nh, const std::string& hand_prefix, const int& joint_nums) {
    joint_nums_ = joint_nums;
    joint_state_sub_ = nh->subscribe("/cb_" + hand_prefix + "_hand_state", 1, &hand_comm::hand_joint_state_cb, this);
    // touch_state_sub_ = nh->subscribe("/cb_" + hand_prefix + "_hand_touch", 1, &hand_comm::hand_touch_state_cb, this);
    joint_state_pub_ = nh->advertise<sensor_msgs::JointState>("/cb_" + hand_prefix + "_hand_control_cmd", 1);

    curangle_.resize(joint_nums, 0);
    curspeed_.resize(joint_nums, 0);
    curforce_.resize(joint_nums, 0);

    if(joint_nums == 10) {
        /*
        'thumb_joint0' 'thumb_joint1' 'thumb_joint2' 'thumb_joint3(mim)' 'thumb_joint4(mim)' 'thumb_joint5'
        'index_joint0' 'index_joint1(mim)' 'index_joint2(mim)' 'index_joint3(mim)' 'index_joint3'
        'middle_joint0' 'middle_joint1(mim)' 'middle_joint2(mim)' 'middle_joint3'
        'ring_joint0' 'ring_joint1(mim)' 'ring_joint2(mim)' 'ring_joint3(mim)' 'ring_joint4'
        'little_joint0' 'little_joint1(mim)' 'little_joint2(mim)' 'little_joint3(mim)' 'little_joint4'
        */
    }



}

bool hand_comm::set_angle(const std::vector<double>& angles) {
    if(angles.size() != joint_nums_) {
        ROS_ERROR_STREAM("Joint number incorrect, should be " << joint_nums_ << " but got " << angles.size());
        return false;
    }
    sensor_msgs::JointState hand_joint_state;
    for (size_t i = 0; i < angles.size(); i++) {
        double encoded_value = (angles[i] / angle_upper_limit[i] - angle_lower_limit[i]) * (angle_encoder_upper_limit[i] - angle_encoder_lower_limit[i]);
        hand_joint_state.position.push_back(encoded_value);
    }

    joint_state_pub_.publish(hand_joint_state);
    return true;
}

void hand_comm::hand_joint_state_cb(const sensor_msgs::JointStateConstPtr& msg) {
    if(msg->position.size() != joint_nums_) {
        ROS_ERROR_STREAM("Joint number incorrect, should be " << joint_nums_ << " but got " << msg->position.size());
    }
    for (size_t i = 0; i < msg->position.size(); i++) {
        curangle_[i] = msg->position[i] * angle_upper_limit[i] - angle_lower_limit[i] / (angle_encoder_upper_limit[i] - angle_encoder_lower_limit[i]);
    }
}

}  // namespace linker_hand

#endif  // HAND_CONTROL_LIB_LINKER_CPP

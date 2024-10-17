#include <inspire_hand/hardware_interface.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

namespace inspire_hand
{
bool HardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
    inspire_hand_.set_nh(&root_nh);
   // connect and register the joint state interface
    for (std::size_t i = 0; i < inspire_hand_.joint_names_.size(); ++i) {
        // Create joint state interface for all joints
        jnt_state_interface_.registerHandle(hardware_interface::JointStateHandle(inspire_hand_.joint_names_[i], &inspire_hand_.curangle_[i], &inspire_hand_.curangle_[i], &inspire_hand_.curforce_[i]));
        jnt_pos_interface_.registerHandle(hardware_interface::JointHandle(jnt_state_interface_.getHandle(inspire_hand_.joint_names_[i]), &inspire_hand_.setangle_[i]));
    }
    // Register callbacks for trajectory passthrough
    jnt_traj_interface_.registerGoalCallback(
        std::bind(&HardwareInterface::startJointInterpolation, this, std::placeholders::_1));
    jnt_traj_interface_.registerCancelCallback(std::bind(&HardwareInterface::cancelJointInterpolation, this));

    set_mode_srv_ = robot_hw_nh.advertiseService<SetMode::Request, SetMode::Response>(
        "set_hand_mode", [&](SetMode::Request& req, SetMode::Response& resp) {
            if(req.mode == (int)control_mode_ && control_mode_ == ControlMode::FOLLOW_JOINT_TRAJECTORY && !start_trajectory_) {
                if(req.angle.size() == 6) {
                    for(int i = 0; i < 6; i++) {
                        inspire_hand_.setangle_[i] = req.angle[i];
                    }
                }
                resp.success = true;
                return true;
            }
            if(req.mode != (int)control_mode_ && control_mode_ == ControlMode::FOLLOW_JOINT_TRAJECTORY && start_trajectory_) {
                resp.success = false;
                return true;
            }
            if(req.mode == (int)ControlMode::FORCE_MODE) {
                if(req.force.size() == 6) {
                    for(int i = 0; i < 6; i++) {
                        inspire_hand_.setforce_[i] = req.force[i];
                    }
                    inspire_hand_.set_force(inspire_hand_.setforce_);
                }
                if(req.preset_thumb_yaw_angle >= angle_lower_limit[5] && req.preset_thumb_yaw_angle <= angle_upper_limit[5]) {
                    inspire_hand_.setangle_[5] = req.preset_thumb_yaw_angle;
                }
                force_state_ = req.force_state;
                force_state_timeout_ = req.force_state_timeout == 0 ? 5.0 : req.force_state_timeout;
                start_trajectory_time_ = ros::Time::now().toSec();
            }
            control_mode_ = (ControlMode)req.mode;
            resp.success = true;
            return true;
        });

    get_grasp_status_srv_ = robot_hw_nh.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
        "get_grasp_status", [&](std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
            for(int i = 0; i < 5; i++) {
                if(inspire_hand_.curforce_[i] > inspire_hand_.setforce_[i]) {
                    resp.success = true;
                    return true;
                }
            }
            resp.success = false;
            return true;
        });
    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_interface_);
    registerInterface(&jnt_traj_interface_);

    for (size_t i = 0; i < 6; i++) {
        inspire_hand_.setforce_[i] = 500;
    }
    inspire_hand_.set_force(inspire_hand_.setforce_);
    return true;
}

void HardwareInterface::startJointInterpolation(const hardware_interface::JointTrajectory& trajectory)
{
    control_mode_ = ControlMode::FOLLOW_JOINT_TRAJECTORY;
    trajectory_ = trajectory;
    trajectory_index_ = 0;
    start_trajectory_ = true;
    start_trajectory_time_ = ros::Time::now().toSec();
    start_point_.clear();
    for(int i = 0; i < 6; i++)
        start_point_.push_back(inspire_hand_.curangle_[i]);
    prev_time_ = 0;
    ROS_INFO("Starting joint-based trajectory forward");
}

void HardwareInterface::cancelJointInterpolation() {
    for(int i = 0; i < 6; i++) {
        inspire_hand_.setangle_[i] = inspire_hand_.curangle_[i];
    }
    start_trajectory_ = false;
    jnt_traj_interface_.setDone(hardware_interface::ExecutionState::PREEMPTED);
}

void HardwareInterface::read(const ros::Time& time, const ros::Duration& period) {
    inspire_hand_.get_actual_angle();
    inspire_hand_.get_actual_force();
    if(start_trajectory_) {
        control_msgs::FollowJointTrajectoryFeedback feedback = control_msgs::FollowJointTrajectoryFeedback();
        for (size_t i = 0; i < 6; i++) {
            feedback.desired.positions.push_back(inspire_hand_.setangle_[i]);
            feedback.actual.positions.push_back(inspire_hand_.curangle_[i]);
            feedback.error.positions.push_back(std::abs(inspire_hand_.curangle_[i] - inspire_hand_.setangle_[i]));
        }
        jnt_traj_interface_.setFeedback(feedback);
    }
}

void HardwareInterface::write(const ros::Time& time, const ros::Duration& period) {
    if(control_mode_ == ControlMode::FOLLOW_JOINT_TRAJECTORY) {
        //Set desired position
        if (start_trajectory_) {
            if (time.toSec() - start_trajectory_time_ > trajectory_.trajectory.points.back().time_from_start.toSec()) {
                start_trajectory_ = false;
                jnt_traj_interface_.setDone(hardware_interface::ExecutionState::SUCCESS);
                return;
            }
            double current_time = time.toSec() - start_trajectory_time_;
            while(trajectory_index_ < trajectory_.trajectory.points.size()) {
                trajectory_msgs::JointTrajectoryPoint point = trajectory_.trajectory.points[trajectory_index_];
                double next_time = point.time_from_start.toSec();
                if (current_time > next_time) {
                    start_point_ = point.positions;
                    prev_time_ = next_time;
                    trajectory_index_++;
                } else {
                    for(int i = 0; i < 6; i++) {
                        inspire_hand_.setangle_[i] = (point.positions[i] - start_point_[i]) * (current_time - prev_time_) / (next_time - prev_time_) + start_point_[i];
                    }
                    break;
                }
            }
        }
    } else if(control_mode_ == ControlMode::FREEDRIVE)  {
        const std::vector<double> force_ratio_lookup = {0.00147, 0.00147, 0.00147, 0.00147, 0.0006, 0.001308};
        const std::vector<double> force_pos_threshold_lookup = {80, 80, 80, 80, 80, 80};
        const std::vector<double> force_neg_threshold_lookup = {-40, -40, -40, -40, -20, -80};
        for(int i = 0; i < 6; i++) {
            if(fabs(inspire_hand_.setangle_[i] - inspire_hand_.curangle_[i]) < 0.05) {
                if(inspire_hand_.curforce_[i] > force_pos_threshold_lookup[i]) {
                    inspire_hand_.setangle_[i] = std::max(inspire_hand_.setangle_[i] - force_ratio_lookup[i] * (inspire_hand_.curforce_[i] - force_pos_threshold_lookup[i]) / 2.0 , angle_lower_limit[i]);
                } else if(inspire_hand_.curforce_[i] < force_neg_threshold_lookup[i]) {
                    inspire_hand_.setangle_[i] = std::min(inspire_hand_.setangle_[i] - force_ratio_lookup[i] * (inspire_hand_.curforce_[i] - force_neg_threshold_lookup[i]), angle_upper_limit[i]);
                }
            }
        }
    } else if(control_mode_ == ControlMode::FORCE_MODE) {
        //Set desired force
        if(force_state_) {
            if(fabs(inspire_hand_.setangle_[5] - inspire_hand_.curangle_[5]) < 0.05 ||
               time.toSec() - start_trajectory_time_ < force_state_timeout_) {
                for(int i = 0; i < 5; i++) {
                    inspire_hand_.setangle_[i] = angle_upper_limit[i];
                }
                control_mode_ = ControlMode::FOLLOW_JOINT_TRAJECTORY;
            }
        } else {
            for(int i = 0; i < 5; i++) {
                inspire_hand_.setangle_[i] = angle_lower_limit[i];
            }
            control_mode_ = ControlMode::FOLLOW_JOINT_TRAJECTORY;
        }
    }
    inspire_hand_.set_angle(inspire_hand_.setangle_);
}
} // namespace inspire_hand
/*********************************************************************************************//**
* hardware_interface.h

* ROS Hardware Interface for the Inspire Hand

* October 2024
* Author:Shuo Liu
* *********************************************************************************************/

#ifndef HARDWARE_INTERFACE_H_INCLUDED
#define HARDWARE_INTERFACE_H_INCLUDED

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <pass_through_controllers/trajectory_interface.h>

#include <inspire_hand/SetMode.h>
#include <std_srvs/Trigger.h>

#include <inspire_hand/hand_control_lib.h>

namespace inspire_hand
{
/*!
 * \brief The HardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class HardwareInterface : public hardware_interface::RobotHW
{
public:
  /*!
   * \brief Creates a new HardwareInterface object.
   */
  HardwareInterface() {};
  virtual ~HardwareInterface() = default;
  /*!
   * \brief Handles the setup functionality for the ROS interface. This includes parsing ROS
   * parameters, creating interfaces, starting the main driver and advertising ROS services.
   *
   * \param root_nh Root level ROS node handle
   * \param robot_hw_nh ROS node handle for the robot namespace
   *
   * \returns True, if the setup was performed successfully
   */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /*!
   * \brief Read method of the control loop. Reads a RTDE package from the robot and handles and
   * publishes the information as needed.
   *
   * \param time Current time
   * \param period Duration of current control loop iteration
   */
  virtual void read(const ros::Time& time, const ros::Duration& period) override;
  /*!
   * \brief Write method of the control loop. Writes target joint positions to the robot to be read
   * by its URCaps program.
   *
   * \param time Current time
   * \param period Duration of current control loop iteration
   */
  virtual void write(const ros::Time& time, const ros::Duration& period) override;

    /*!
   * \brief Getter for the current control frequency
   *
   * \returns The used control frequency
   */
  uint32_t getControlFrequency() {
    return 60;
  };

  bool shouldResetControllers() {
    return false;
  };

  void startJointInterpolation(const hardware_interface::JointTrajectory& trajectory);
  void cancelJointInterpolation();

private:
    inspire_hand::hand_serial inspire_hand_;
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;
    hardware_interface::JointTrajectoryInterface jnt_traj_interface_;

    enum class ControlMode : int {
        FOLLOW_JOINT_TRAJECTORY = 0,
        FREEDRIVE,
        FORCE_MODE,
    } control_mode_{ControlMode::FOLLOW_JOINT_TRAJECTORY};

    ros::ServiceServer set_mode_srv_, get_grasp_status_srv_;

    hardware_interface::JointTrajectory trajectory_;
    int trajectory_index_;
    bool start_trajectory_;
    double start_trajectory_time_;
    double prev_time_;
    std::vector<double> start_point_;
    bool force_state_;
    double force_state_timeout_;
};
}

#endif  // ifndef INSPIRE_HAND_HARDWARE_INTERFACE_H_INCLUDED
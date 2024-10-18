/*********************************************************************************************//**
* hand_control_lib.h

* This file contains the class definition for the hand_serial class. This class is used to
* communicate with the Inspire Hand via serial communication.

* October 2024
* Author:Shuo Liu

* *********************************************************************************************/



#ifndef HAND_CONTROL_LIB_H
#define HAND_CONTROL_LIB_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <mutex>

namespace inspire_hand {
// Define the maximum and minimum angle limits for the hand
const double angle_upper_limit[] = {1.47, 1.47, 1.47, 1.47, 0.6, 1.308};
const double angle_lower_limit[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

class hand_serial {
public:

    hand_serial() {};
    hand_serial(ros::NodeHandle *nh);

    ~hand_serial();
    void set_nh(ros::NodeHandle *nh);

    int connect();

    //Send and receive data via register
    bool set_reg(const int value, uint8_t pin1, uint8_t pin2, double delay=0.005);
    bool set_reg(const double values[6], uint8_t pin1, uint8_t pin2, double delay=0.005);
    void get_reg(double (&values)[6], uint8_t pin1, uint8_t pin2, bool bit7=false, double delay=0.005);
    unsigned int check_sum(const std::vector<uint8_t>& output);

    //Set the id of the hand
    bool set_id(int id);

    //Set the baudrate of the hand
    bool set_redu_ratio(int redu_ratio);

    //Clear the error of the hand
    bool set_clear_error();

    //Save the parameters to flash
    bool set_save_flash();

    //Reset to factory settings
    bool set_reset_parameters();

    //Set the force calibration
    bool set_force_calibration();

    //Set gesture number
    bool set_gesture_number(int gesture_no);

    //Set current limit
    bool set_current_limit(const double current_limit[6]);

    //Set the hand to the default speed
    bool set_default_speed(const double speed[6]);

    //Set the hand to the default force
    bool set_default_force(const double force[6]);

    //Set the hand to the default angle
    bool set_user_defined_angle(const double angle[6], int k);

    //Set position of the hand (0-2000)
    bool set_position(const double pos[6]);

    //Set the angle of the hand (limited by definition above)
    bool set_angle(const double angle[6]);

    //Set force threshold (> will stop)
    bool set_force(const double force[6]);

    //Set speed of the hand (0-1000)
    bool set_speed(const double speed[6]);

    //Get the actual position of the hand
    void get_actual_position();

    //Get the actual angle of the hand
    void get_actual_angle();

    //Get the actual force of the hand
    void get_actual_force();

    //Get the actual current of the hand
    void get_actual_current();

    //Get the error of the hand
    uint8_t get_error();

    //Get the status of the hand
    void get_status();

    //Get the temperature of the hand
    void get_temp();

    //Get set position
    void get_set_position();

    //Get set angle
    void get_set_angle();

    //Get set force
    void get_force_set();

    /** \brief Set periodic position reading by GET_STATE(0x95) command */
    //void getPeriodicPositionUpdate(double update_frequency);

    /** \brief Function to determine checksum*/
    uint16_t CRC16(uint16_t crc, uint16_t data);

    /** \brief Conversion from 4 bytes to double*/
    double IEEE_754_to_double(uint8_t *raw);

    /** \brief Conversion from double to 4 bytes*/
    void double_to_IEEE_754(double position, unsigned int *output_array);

    //Launch params
    int hand_id_;
    std::string port_name_;
    int baudrate_;

    //hand state variables
    double act_position_;
    uint8_t hand_state_;
    double curpos_[6];
    double curangle_[6];
    double curforce_[6];
    double current_[6];
    double errorvalue_[6];
    double statusvalue_[6];
    double tempvalue_[6];
    double setpos_[6];
    double setangle_[6];
    double setforce_[6];
    std::vector<std::string> joint_names_;
    //sensor_msgs::JointState hand_joint_state_;

    //Serial variables
    serial::Serial *com_port_;

private:
    //Consts

    std::mutex cmd_mutex_;
    //static const double MIN_GRIPPER_VEL_LIMIT = 0;
    //static const double MAX_GRIPPER_VEL_LIMIT = 83;
    //static const double MIN_GRIPPER_ACC_LIMIT = 0;
    //static const double MAX_GRIPPER_ACC_LIMIT = 320;
    static constexpr double WAIT_FOR_RESPONSE_INTERVAL = 0.5;
    static constexpr double INPUT_BUFFER_SIZE = 64;
    //static const int    URDF_SCALE_FACTOR = 2000;

};
}

#endif

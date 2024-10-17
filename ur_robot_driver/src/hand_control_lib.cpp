#ifndef HAND_CONTROL_LIB_CPP
#define HAND_CONTROL_LIB_CPP

#include <inspire_hand/hand_control_lib.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <string>

namespace inspire_hand {
hand_serial::hand_serial(ros::NodeHandle *nh):
    act_position_(-1),
    hand_state_(0xff) {
    set_nh(nh);
}

void hand_serial::set_nh(ros::NodeHandle *nh) {
    //Read launch file params
    nh->getParam("inspire_hand/hand_id", hand_id_);
    nh->getParam("inspire_hand/portname", port_name_);
    nh->getParam("inspire_hand/baudrate", baudrate_);
    nh->getParam("inspire_hand/joints", joint_names_);

    //Initialize and open serial port
    com_port_ = new serial::Serial(port_name_, (uint32_t)baudrate_, serial::Timeout::simpleTimeout(5));
    if (com_port_->isOpen()) {
        ROS_INFO_STREAM("Hand: Serial port " << port_name_ << " openned");
        int id_state = 0;
        while (true) {
            id_state = connect();
            if (id_state == 1) break;
            hand_id_++;
            if (hand_id_ >= 256) {
                ROS_INFO("Failed to connect to hand, restarting from beginning");
                hand_id_ = 1;
            }
        }

        //Get initial state and discard input buffer
        while (hand_state_ == 0xff) {
            //hand_state_ = 0x01;
            hand_state_ = get_error();
            ros::Duration(WAIT_FOR_RESPONSE_INTERVAL).sleep();
        }
    } else {
        ROS_ERROR_STREAM("Hand: Serial port " << port_name_ << " not opened");
    }
}

hand_serial::~hand_serial() {
    com_port_->close();      //Close port
    delete com_port_;        //delete object
}

bool hand_serial::set_reg(const int value, uint8_t pin1, uint8_t pin2, double delay) {
    std::lock_guard<std::mutex> lk(cmd_mutex_);

    std::vector<uint8_t> output {0xEB, 0x90, hand_id_, 0x04, 0x12, pin1, pin2};

    unsigned int temp_int;
    temp_int = (unsigned int)value;
    output.push_back(temp_int);

    //Add checksum 
    output.push_back(check_sum(output) & 0xff);

    //Send message to the module
    com_port_->write(output);

    ros::Duration(delay).sleep();

    //Read response
    std::vector<uint8_t> input;
    while (input.empty()) {
        com_port_->read(input, (size_t)64);
    }
    return input[7];
}

bool hand_serial::set_reg(const double values[6], uint8_t pin1, uint8_t pin2, double delay) {
    std::lock_guard<std::mutex> lk(cmd_mutex_);

    std::vector<uint8_t> output {0xEB, 0x90, hand_id_, 0x0F, 0x12, pin1, pin2};

    for(int i = 0; i < 6; i++) {
        unsigned int temp_int;
        temp_int = (unsigned int)values[i];
        output.push_back(temp_int & 0xff);
        output.push_back((temp_int >> 8) & 0xff);
    }
    //Add checksum 
    output.push_back(check_sum(output) & 0xff);

    //Send message to the module
    com_port_->write(output);

    ros::Duration(delay).sleep();

    //Read response
    std::vector<uint8_t> input;
    while (input.empty()) {
        com_port_->read(input, (size_t)64);
    }
    return input[7];
}

void hand_serial::get_reg(double (&values)[6], uint8_t pin1, uint8_t pin2, bool bit7, double delay) {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> output ={0xEB, 0x90, hand_id_, 0x04, 0x11, pin1, pin2, bit7 ? 0x06:0x0C};

    //Add checksum 
    output.push_back(check_sum(output) & 0xff);
    //Send message to the module
    com_port_->write(output);

    ros::Duration(delay).sleep();

    //Read response
    std::vector<uint8_t> input;
    while (input.empty()) {
        com_port_->read(input, (size_t)64);
    }

    if(bit7) {
        for (int j = 0; j<6; j++)
            values[j] = double(input[7 + j]);
    } else {
        for (int j = 0; j<6; j++)
            values[j] = double(((input[8 + j * 2] << 8) & 0xff00) + input[7 + j * 2]);
    }
}

unsigned int hand_serial::check_sum(const std::vector<uint8_t>& output) {
    unsigned int check_num = 0;
    int len = output[3] + 5;
    for (int i = 2; i < len - 1; i++)
        check_num = check_num + output[i];
    return check_num;
}

int hand_serial::connect() {
    std::vector<uint8_t> output;
    output.push_back(0xEB);
    output.push_back(0x90);
    output.push_back(hand_id_);
    output.push_back(0x04);
    output.push_back(0x11);
    output.push_back(0xFE);
    output.push_back(0x05);
    output.push_back(0x0C);

    unsigned int check_num = 0;

    int len = output[3] + 5;
    for (int i = 2; i < len - 1; i++)
        check_num = check_num + output[i];

    //Add checksum to the output buffer
    output.push_back(check_num & 0xff);

    //Send message to the module and wait for response
    com_port_->write(output);

    ros::Duration(0.001).sleep();

    //Read response
    std::vector<uint8_t> input;
    com_port_->read(input, (size_t)64);
    //ROS_INFO("ok");
    if (input.empty())
        return 0;
    else
        return 1;
}

bool hand_serial::set_id(int id) {
    hand_id_ = id;
    return set_reg(id, 0xE8, 0x03);
}

bool hand_serial::set_redu_ratio(int redu_ratio) {
    if (redu_ratio == 0)
        baudrate_ = 115200;
    else if (redu_ratio == 1)
        baudrate_ = 57600;
    else
        baudrate_ = 19200;
    return set_reg(redu_ratio, 0xE9, 0x03);
}

bool hand_serial::set_clear_error() {
    return set_reg(0x01, 0xEC, 0x03, 1.0);
}

bool hand_serial::set_save_flash() {
    return set_reg(0x01, 0xED, 0x03, 1.0);
}

bool hand_serial::set_reset_parameters() {
    return set_reg(0x01, 0xEE, 0x03, 1.0);
}

bool hand_serial::set_force_calibration() {
    return set_reg(0x01, 0xF1, 0x03, 1.0);
}

bool hand_serial::set_gesture_number(int gesture_no) {
    return set_reg(gesture_no, 0xF0, 0x03, 1.0);
}

bool hand_serial::set_current_limit(const double current_limits[6]) {
    return set_reg(current_limits, 0xFC, 0x03);
}

bool hand_serial::set_default_speed(const double speed[6]) {
    return set_reg(speed, 0x08, 0x04);
}

bool hand_serial::set_default_force(const double force[6]) {
    return set_reg(force, 0x14, 0x04);
}

bool hand_serial::set_user_defined_angle(const double angle[6], int k) {
    int temp;
    temp = 1066 + (k - 14) * 12;
    unsigned int temp_int;
    temp_int = (unsigned int)temp;
    double encoder[6];
    for(int i = 0; i < 6; i++) {
        encoder[i] = (1000.0 - 1000.0 * angle[i] / (angle_upper_limit[i] - angle_lower_limit[i]));
    }
    return set_reg(encoder, temp_int & 0xff, (temp_int >> 8) & 0xff);
}

bool hand_serial::set_position(const double pos[6]) {
    return set_reg(pos, 0xC2, 0x05);
}

bool hand_serial::set_angle(const double angle[6]) {
    double encoder[6];
    for(int i = 0; i < 6; i++) {
        encoder[i] = (1000.0 - 1000.0 * angle[i] / (angle_upper_limit[i] - angle_lower_limit[i]));
    }
    return set_reg(encoder, 0xCE, 0x05);
}

bool hand_serial::set_force(const double force[6]) {
    return set_reg(force, 0xDA, 0x05);
}

bool hand_serial::set_speed(const double speed[6]) {
    return set_reg(speed, 0xF2, 0x05);
}

void hand_serial::get_actual_position() {
    get_reg(curpos_, 0xFE, 0x05);
}

void hand_serial::get_actual_angle() {
    double angle[6];
    get_reg(angle, 0x0A, 0x06);
    for(int i = 0; i<6; i++) {
        curangle_[i] = (1000.0 - angle[i]) / 1000.0 * (angle_upper_limit[i] - angle_lower_limit[i]);
    }
}

void hand_serial::get_actual_force() {
    double force[6];
    get_reg(force, 0x2E, 0x06);
    for(int i = 0; i<6; i++)
        curforce_[i] = force[i]>32768?force[i]-65536:force[i];
}

void hand_serial::get_actual_current() {
    get_reg(current_, 0x3A, 0x06);
}

uint8_t hand_serial::get_error() {
    get_reg(errorvalue_, 0x46, 0x06, true);
    if (errorvalue_[0] == 0 && errorvalue_[1] == 0 &&errorvalue_[2] == 0 &&errorvalue_[3] == 0 &&errorvalue_[4] == 0 &&errorvalue_[5] == 0)
        return((uint8_t)0x00);
    else
        return((uint8_t)0xff);
}

void hand_serial::get_status() {
    get_reg(statusvalue_, 0x4C, 0x06, true);
}

void hand_serial::get_temp() {
    get_reg(tempvalue_, 0x52, 0x06, true);
}

void hand_serial::get_set_position() {
    get_reg(setpos_, 0xC2, 0x05);
}

void hand_serial::get_set_angle() {
    double angle[6];
    get_reg(angle, 0xCE, 0x05);
    for(int i = 0; i<6; i++) {
        setangle_[i] = (1000.0 - angle[i]) / 1000.0 * (angle_upper_limit[i] - angle_lower_limit[i]);
    }
}

void hand_serial::get_force_set() {
    get_reg(setforce_, 0xDA, 0x05);
}
}

#endif

#ifndef HAND_CONTROL_WRAPPER_H
#define HAND_CONTROL_WRAPPER_H

#include <memory>
#include <ros/ros.h>
#include <modbus/modbus.h>
#undef TRUE
#undef FALSE
#include <sensor_msgs/JointState.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>

namespace inspire_hand {
// Base class for hand control
class HandControlBase {
public:
    virtual ~HandControlBase() = default;

    // Common interface methods
    virtual void initialize(int hand_id, const std::string& connection_param, int extra_param) = 0;
    virtual bool get_error() = 0;
    virtual bool get_actual_force() = 0;
    virtual bool get_actual_current() = 0;
    virtual bool get_actual_angle() = 0;
    virtual bool get_actual_position() = 0;
    virtual bool get_status() = 0;
    virtual bool get_temp() = 0;
    virtual bool get_set_position() = 0;
    virtual bool get_set_angle() = 0;
    virtual bool get_set_force() = 0;
    virtual bool set_clear_error() = 0;
    virtual bool set_id(int id) = 0;
    virtual bool set_position(const double pos[6]) = 0;
    virtual bool set_angle(const double angle[6]) = 0;
    virtual bool set_force(const double force[6]) = 0;
    virtual bool set_speed(const double speed[6]) = 0;
    virtual bool set_force_calibration() = 0;
    virtual bool set_reset_parameters() = 0;
    virtual bool set_current_limit(const double current_limit[6]) = 0;
    virtual bool set_default_speed(const double speed[6]) = 0;
    virtual bool set_default_force(const double force[6]) = 0;
    virtual bool set_gesture_number(int gesture_no) = 0;
    virtual bool set_redu_ratio(int redu_ratio) = 0;

    // Common state variables
    int hand_id_;
    double curpos_[6];
    double curangle_[6]; 
    double curspeed_[6] = {0.0};
    double curforce_[6];
    double current_[6];
    double setpos_[6];
    double setangle_[6];
    double setangle_cmd_[6];
    double setforce_[6];
    double setspeed_[6];
    double errorvalue_[6];
    double statusvalue_[6];
    double tempvalue_[6];
    int32_t safety_mode = 1;
    std::vector<double> angle_upper_limit;
    std::vector<double> angle_lower_limit;
    std::vector<double> force_pos_threshold_lookup;
    std::vector<double> force_neg_threshold_lookup;

protected:
    std::mutex cmd_mutex_;
};

// TCP/Modbus implementation
class HandControlFTP : public HandControlBase {
public:
    HandControlFTP() {
        angle_upper_limit = {1.6, 1.6, 1.6, 1.6, 0.92, 1.7};
        angle_lower_limit = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        force_pos_threshold_lookup = {80, 80, 80, 80, 80, 80};
        force_neg_threshold_lookup = {-20, -20, -20, -20, -10, -80};
    };
    ~HandControlFTP();

    void initialize(int hand_id, const std::string& ip_address, int port) override;
    
    // Implement all virtual methods from base class
    bool get_error() override;
    bool get_actual_force() override;
    bool get_actual_current() override;
    bool get_actual_angle() override;
    bool get_actual_position() override;
    bool get_status() override;
    bool get_temp() override;
    bool get_set_position() override;
    bool get_set_angle() override;
    bool get_set_force() override;
    bool set_clear_error() override;
    bool set_id(int id) override;
    bool set_position(const double pos[6]) override;
    bool set_angle(const double angle[6]) override;
    bool set_force(const double force[6]) override;
    bool set_speed(const double speed[6]) override;
    bool set_force_calibration() override;
    bool set_reset_parameters() override;
    bool set_current_limit(const double current_limit[6]) override;
    bool set_default_speed(const double speed[6]) override;
    bool set_default_force(const double force[6]) override;
    bool set_gesture_number(int gesture_no) override;
    bool set_redu_ratio(int redu_ratio) override;

    // Additional TCP-specific methods
    bool get_tactile_data();
    
private:
    modbus_t* ctx_;
    std::string ip_address_;
    int port_;
    
    cv::Mat convert_tactile_data_to_image(const std::vector<std::vector<std::vector<uint16_t>>>& data, 
                                         int rows=256, int cols=256);
    std::vector<std::vector<uint16_t>> resize_tactile_data(uint16_t* v, int rows, int cols);
    
    int readRegister(int reg_addr);
    int readRegisters(int reg_addr, int num_registers, uint16_t* tab_reg);
    int writeRegister(int reg_addr, int value);
    int writeMultipleRegisters(int start_addr, const uint16_t* values, int num_values);
    int writeMultipleRegisters(int start_addr, const double* values, int num_values);

    bool validate_values(const double values[6], double lower_limit, double upper_limit);
    bool validate_values(const uint16_t values[6], uint16_t lower_limit, uint16_t upper_limit);
    bool save_setting();

public:
    std::vector<std::vector<std::vector<uint16_t>>> multi_tactile_data_;
    cv::Mat multi_tactile_image_;
};

// Serial implementation  
class HandControlSerial : public HandControlBase {
public:
    HandControlSerial() {
        angle_upper_limit = {1.47, 1.47, 1.47, 1.47, 0.6, 1.308};
        angle_lower_limit = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        force_pos_threshold_lookup = {80, 80, 80, 80, 80, 80};
        force_neg_threshold_lookup = {-60, -60, -60, -60, -10, -220};
    };
    ~HandControlSerial();

    void initialize(int hand_id, const std::string& port_name, int baudrate) override;

    // Implement all virtual methods from base class
    bool get_error() override;
    bool get_actual_force() override;
    bool get_actual_current() override;
    bool get_actual_angle() override;
    bool get_actual_position() override;
    bool get_status() override;
    bool get_temp() override;
    bool get_set_position() override;
    bool get_set_angle() override;
    bool get_set_force() override;
    bool set_clear_error() override;
    bool set_id(int id) override;
    bool set_position(const double pos[6]) override;
    bool set_angle(const double angle[6]) override;
    bool set_force(const double force[6]) override;
    bool set_speed(const double speed[6]) override;
    bool set_force_calibration() override;
    bool set_reset_parameters() override;
    bool set_current_limit(const double current_limit[6]) override;
    bool set_default_speed(const double speed[6]) override;
    bool set_default_force(const double force[6]) override;
    bool set_gesture_number(int gesture_no) override;
    bool set_redu_ratio(int redu_ratio) override;

    // Additional serial-specific methods
    bool set_save_flash();
    bool set_user_defined_angle(const double angle[6], int k);

private:
    serial::Serial* com_port_;
    int baudrate_;

    int connect();
    bool set_reg(const int value, uint8_t pin1, uint8_t pin2, double delay=0.005);
    bool set_reg(const double values[6], uint8_t pin1, uint8_t pin2, double delay=0.005);
    bool get_reg(double (&values)[6], uint8_t pin1, uint8_t pin2, bool bit7=false, double delay=0.005);
    unsigned int check_sum(const std::vector<uint8_t>& output);
    uint16_t CRC16(uint16_t crc, uint16_t data);
    double IEEE_754_to_double(uint8_t* raw);
    void double_to_IEEE_754(double position, unsigned int* output_array);

    static constexpr double WAIT_FOR_RESPONSE_INTERVAL = 0.5;
    static constexpr double INPUT_BUFFER_SIZE = 64;
};

// Factory class for creating hand control instances
class HandControlFactory {
public:
    static std::shared_ptr<HandControlBase> create(bool use_inspire_v2) {
        if (use_inspire_v2) {
            return std::shared_ptr<HandControlBase>(new HandControlFTP());
        } else {
            return std::shared_ptr<HandControlBase>(new HandControlSerial()); 
        }
    }
};

} // namespace inspire_hand

#endif // HAND_CONTROL_WRAPPER_H

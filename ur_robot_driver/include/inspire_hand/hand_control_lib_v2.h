#ifndef HAND_CONTROL_LIB_V2_H
#define HAND_CONTROL_LIB_V2_H

#include <ros/ros.h>
#include <modbus/modbus.h>
#undef TRUE
#undef FALSE
#include <sensor_msgs/JointState.h>

// ...其他服务头文件...
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace inspire_hand {
// Define the maximum and minimum angle limits for the hand
const double angle_upper_limit[] = {1.6, 1.6, 1.6, 1.6, 0.92, 1.7};
const double angle_lower_limit[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
class hand_serial {
public:
    hand_serial() {};

    ~hand_serial();

    void initialize(int hand_id, std::string ip_address, int port);

    bool get_error();
    
    bool get_actual_force();
    
    bool get_actual_current();
    
    bool get_set_angle();
                              
    bool get_actual_angle();
                                 
    bool get_set_force();
    
    bool get_temp();
    
    bool get_set_position();
                            
    bool get_actual_position();
                            
    bool get_status();
                                                                                                                    
    bool set_clear_error();         
                              
    bool set_id(int id);

    bool set_redu_ratio(int redu_ratio);
    
    bool set_position(const double pos[6]);
    
    bool set_gesture_number(int gesture_no);
    
    bool set_speed(const double speed[6]);
                          
    bool set_default_speed(const double speed[6]);
    
    bool set_angle(const double angle[6]);
                      
    bool set_force_calibration();
                              
    bool set_force(const double force[6]); 
                          
    bool set_default_force(const double force[6]);
                                  
    bool set_current_limit(const double current_limit[6]);
    
    bool set_reset_parameters();
                     
    int writeMultipleRegisters(int start_addr, const uint16_t* values, int num_values);
    int getRegisterValue(int reg_addr) {
        return readRegister(reg_addr); // 通过公有方法调用私有方法
    }
    cv::Mat convert_tactile_data_to_image(const std::vector<std::vector<std::vector<int>>>& multi_tactile_data, int rows=256, int cols=256);
    std::vector<std::vector<int>> resize_tactile_data(const std::vector<int>& v, int rows, int cols);
    bool read_tactile(int start_addr, std::vector<int>& tactile_data, int num_values);
    bool get_tactile_data();

    //hand state variables              
    int hand_id_;
    double curpos_[6];
    double curangle_[6];
    double curspeed_[6] = {0.0};
    double curforce_[6];
    double current_[6];
    double errorvalue_[6];
    int statusvalue_[6];
    int tempvalue_[6];
    double setpos_[6];
    double setangle_[6];
    double setangle_cmd_[6];
    double setforce_[6];
    std::vector<std::vector<std::vector<int>>> multi_tactile_data_;
    cv::Mat multi_tactile_image_;

private:
    // Modbus TCP 上下文
    modbus_t *ctx_;
    std::mutex cmd_mutex_;

    // 设备参数
    std::string ip_address_;
    int port_;

    // ...其他设置函数...
    // 读取和写入 Modbus 数据的通用方法
    int readRegister(int reg_addr);
    int readRegisters(int reg_addr, int num_registers, uint16_t *tab_reg);
    int writeRegister(int reg_addr, int value);


    bool validate_values(const double values[6], double lower_limit, double upper_limit);
    bool set_reg(const double values[6], int pin);
    void get_reg(double (&values)[6], int pin);
    bool save_setting();

};

} // namespace inspire_hand

#endif


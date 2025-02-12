#ifndef HAND_CONTROL_LIB_V2_CPP
#define HAND_CONTROL_LIB_V2_CPP

#include <inspire_hand/hand_control_lib_v2.h>
#include <ros/ros.h>

namespace inspire_hand {
// Destructor
hand_serial::~hand_serial()
{
    if (ctx_) {
        modbus_close(ctx_);
        modbus_free(ctx_);
    }
}

void hand_serial::initialize(int hand_id, std::string ip_address, int port) {
    // Initialize Modbus TCP context
    hand_id_ = hand_id;
    ip_address_ = ip_address; // Replace with your device's IP
    port_ = port; // Default Modbus TCP port
    ctx_ = modbus_new_tcp(ip_address_.c_str(), port_);
    
    if (modbus_connect(ctx_) == -1) {
        ROS_ERROR("Unable to connect to Modbus server: %s", modbus_strerror(errno));
        modbus_free(ctx_);
        ctx_ = nullptr;
    }

    ROS_DEBUG("Connected to Modbus server at %s:%d", ip_address_.c_str(), port_);
}

void hand_serial::get_reg(double (&values)[6], int pin) {
    for(int i = 0; i < 6; i++) {
        values[i] = readRegister(pin + i * 2);
    }
}

// Callback to get error information
bool hand_serial::get_error() {
    ROS_DEBUG("Hand: Get error request received");

    uint16_t tab_reg[6]; // 用于存储读取的寄存器值

    // 读取寄存器 (地址从 1606 开始)
    int rc = readRegisters(1606, 6, tab_reg);
    if (rc == -1) {
        ROS_ERROR("Failed to read error registers: %s", modbus_strerror(errno));
        return false; // 返回失败
    }

    // 解析故障信息并存储到响应中
    for (int i = 0; i < 3; i++) {
        // 每个寄存器包含两个字节
        errorvalue_[i * 2] = static_cast<float>(tab_reg[i] & 0xFF);    // 低字节在前
        errorvalue_[i * 2 + 1] = static_cast<float>((tab_reg[i] >> 8) & 0xFF); // 高字节在后
        
        // 输出故障信息
        ROS_DEBUG("ERROR(%d): %f", i * 2, errorvalue_[i * 2]);         // 输出低字节
        ROS_DEBUG("ERROR(%d): %f", i * 2 + 1, errorvalue_[i * 2 + 1]); // 输出高字节
    }

    return true; // 返回成功
}

bool hand_serial::get_status() {
    ROS_DEBUG("Hand: Get status request received");

    uint16_t tab_reg[6]; // 用于存储读取的寄存器值

    // 读取寄存器 (地址从 1612 开始)
    int rc = readRegisters(1612, 6, tab_reg);
    if (rc == -1) {
        ROS_ERROR("Failed to read status registers: %s", modbus_strerror(errno));
        return false; // 返回失败
    }

    // 解析状态信息并存储到响应中
    for (int i = 0; i < 3; i++) {
        // 每个寄存器包含两个字节
        statusvalue_[i * 2] = static_cast<int16_t>(tab_reg[i] & 0xFF);    // 低字节在前
        statusvalue_[i * 2 + 1] = static_cast<int16_t>((tab_reg[i] >> 8) & 0xFF); // 高字节在后
        
        // 输出状态信息
        ROS_DEBUG("STATUS(%d): %d", i * 2, statusvalue_[i * 2]);         // 输出低字节
        ROS_DEBUG("STATUS(%d): %d", i * 2 + 1, statusvalue_[i * 2 + 1]); // 输出高字节
    }

    return true; // 返回成功
}

bool hand_serial::get_actual_force() {
    ROS_DEBUG("Hand: Get Force Actual values request received");

    // 直接读取各个手指的实际受力值
    double force[6];
    get_reg(force, 1582); // 从寄存器 1582 开始读取
    for(int i = 0; i<6; i++)
        curforce_[i] = force[i]>32768?force[i]-65536:force[i];

    return validate_values(curforce_, -4000, 4000);
}

bool hand_serial::get_actual_current() {
    ROS_DEBUG("Hand: Get Current values request received");

    // 直接读取各个电缸的电流值
    get_reg(current_, 1594); // 从寄存器 1594 开始读取

    return validate_values(current_, 0, 1000);
}

bool hand_serial::get_set_angle() {
    ROS_DEBUG("Hand: Get Angle Set values request received");

    double angle[6];
    // 直接读取各个手指的上电初始角度
    get_reg(angle, 1486); // 从寄存器 1486 开始读取

    if (!validate_values(angle, 0, 1000)) {
        return false;
    }

    for(int i = 0; i < 6; i++) {
        setangle_[i] = (1000.0 - angle[i]) / 1000.0 * (angle_upper_limit[i] - angle_lower_limit[i]);
    }
    return true;
}

bool hand_serial::get_actual_angle() {
    ROS_DEBUG("Hand: Get Angle Actual values request received");

    double angle[6];
    // 直接读取各个手指的上电初始角度
    get_reg(angle, 1546); // 从寄存器 1486 开始读取

    if (!validate_values(angle, 0, 1000)) {
        return false;
    }

    for(int i = 0; i < 6; i++) {
        curangle_[i] = (1000.0 - angle[i]) / 1000.0 * (angle_upper_limit[i] - angle_lower_limit[i]);
    }
    return true;
}

bool hand_serial::get_set_force() {
    ROS_DEBUG("Hand: Get Force Set values request received");

    // 直接读取各个手指的力控设置值
    get_reg(setforce_, 1498); // 从寄存器 1498 开始读取

    return validate_values(setforce_, 0, 3000); // 检查读取的值是否有效
}

bool hand_serial::get_temp()
{
    ROS_DEBUG("Hand: Get temperature request received");

    uint16_t tab_reg[6]; // 用于存储读取的寄存器值

    // 读取寄存器 (地址从 1618 开始)
    int rc = readRegisters(1618, 6, tab_reg);
    if (rc == -1) {
        ROS_ERROR("Failed to read temperature registers: %s", modbus_strerror(errno));
        return false; // 返回失败
    }

    // 解析温度值并存储到响应中
    for (int i = 0; i < 3; i++) {
        // 每个寄存器包含两个字节
        tempvalue_[i * 2] = static_cast<int16_t>(tab_reg[i] & 0xFF);       // 低字节;
        tempvalue_[i * 2 + 1] = static_cast<int16_t>((tab_reg[i] >> 8) & 0xFF); // 高字节
        
        // 输出温度信息
        ROS_DEBUG("TEMP(%d): %d", i * 2, tempvalue_[i * 2]);      // 输出低字节
        ROS_DEBUG("TEMP(%d): %d", i * 2 + 1, tempvalue_[i * 2 + 1]); // 输出高字节
    }

    return true; // 返回成功
}

bool hand_serial::get_set_position() {
    ROS_DEBUG("Hand: Get Position Set values request received");

    // 直接读取各个手指的驱动器位置设置值
    get_reg(setpos_, 1474); // 从寄存器 1474 开始读取

    return validate_values(setpos_, 0, 2000); // 检查读取的值是否有效
}

bool hand_serial::get_actual_position() {
    ROS_DEBUG("Hand: Get Position Actual values request received");

    // 直接读取各个手指的驱动器实际位置值
    get_reg(curpos_, 1534); // 从寄存器 1534 开始读取

    return validate_values(curpos_, 0, 2000); // 检查读取的值是否有效
}

// Callback to set ID
bool hand_serial::set_id(int id) {
    ROS_DEBUG("Hand: Set ID request received");

    // 检查请求中的 ID 是否在合法范围内
    if (id >= 1 && id <= 254) {
        // 将 ID 写入 Modbus 寄存器
        int write_result = writeRegister(1000, id);
        
        if (write_result == 0) {
            // 读取寄存器以验证写入的 ID
            int read_value = readRegister(1000);
            if (read_value != -1) {
                ROS_DEBUG("Read ID from Modbus register: %d", read_value);
                if (read_value == id) {
                    ROS_DEBUG("ID verification successful.");
                } else {
                    ROS_WARN("ID verification failed! Expected: %d, Read: %d", id, read_value);
                    return false;
                }
            }
        }
    }
    return true; // 返回成功
}

// Callback to set Reduction Ratio
bool hand_serial::set_redu_ratio(int redu_ratio) {
    ROS_DEBUG("Hand: Set Reduction Ratio request received");

    // 检查请求中的 redu_ratio 是否在合法范围内
    if (redu_ratio >= 0 && redu_ratio <= 4) {
        // 将 redu_ratio 写入 Modbus 寄存器
        int write_result = writeRegister(1002, redu_ratio);
        
        if (write_result == 0) {
            // 读取寄存器以验证写入的 redu_ratio
            int read_value = readRegister(1002);
            if (read_value != -1) {
                ROS_DEBUG("Read REDU_RATIO from Modbus register: %d", read_value);
                if (read_value == redu_ratio) {
                    ROS_DEBUG("Reduction Ratio verification successful.");
                } else {
                    ROS_WARN("Reduction Ratio verification failed! Expected: %d, Read: %d", redu_ratio, read_value);
                    return false;
                }
            } 
            return true; // 返回成功
        }
    }
}

bool hand_serial::set_gesture_number(int gesture_no)
{
    int register_address = 0x0910;  // 当前动作序列索引寄存器
    int action_register_address = 0x0912; // 动作序列号寄存器地址

    // 首先将手势编号写入 Modbus 寄存器 2320
    if (writeRegister(register_address, gesture_no) == -1) {
        ROS_ERROR("Failed to set gesture number: %d", gesture_no);
        return false; // 写入失败
    }

    // 然后写入执行动作序列号的寄存器 2322
    if (writeRegister(action_register_address, 1) == -1) {
        ROS_ERROR("Failed to execute action sequence for gesture number: %d", gesture_no);
        return false; // 写入失败
    }

    return true; // 写入成功
}

bool hand_serial::validate_values(const double values[6], double lower_limit, double upper_limit) {
    // 检查请求中的位置参数是否合法
    for(int i = 0; i < 6; i++) {
        if (values[i] < lower_limit || values[i] > upper_limit) {
            ROS_WARN("Hand: value error! Values (%f) must be >= %f and <= %f.", values[i], lower_limit, upper_limit);
            return false; // 返回失败
        }
    }
    return true; // 返回成功
}

bool hand_serial::set_reg(const double values[6], int pin) {
    // 将位置值写入 Modbus 寄存器
    bool status = true;
    for(int i = 0; i < 6; i++) {
        status = status && (writeRegister(pin + i * 2, values[i]) == 0);
    }
    // 读取某个寄存器的值（位置0）
    int read_value = readRegister(pin);
    if (read_value != -1) {
        ROS_DEBUG("Read value: %d", read_value);
    }
    return status; // 返回成功
}

bool hand_serial::save_setting() {
    // 写入寄存器 1005 以保存设置
    uint16_t save_value = 1; // 代表保存设置
    int save_rc = writeRegister(1005, save_value);
    if (save_rc == -1) {
        ROS_ERROR("Failed to write to Modbus register 1005 to save settings");
        return false; // 返回失败
    }
    return true; // 返回成功
}

// Callback to set position
bool hand_serial::set_position(const double pos[6]) {
    ROS_DEBUG("hand: set pos");
    if(validate_values(pos, 0, 2000)) {
        return set_reg(pos, 1474); // 返回成功
    }
    return false; // 返回失败
}

bool hand_serial::set_speed(const double speed[6]) {
    ROS_DEBUG("hand: set speed");

    if(validate_values(speed, 0, 1000)) {
        return set_reg(speed, 1522); // 返回成功
    }
}

bool hand_serial::set_default_speed(const double speed[6]) {
    ROS_DEBUG("hand: set default speed");
    if(validate_values(speed, 0, 1000) && set_reg(speed, 1032)) {
        return save_setting(); // 返回成功
    }
    return false; // 返回失败
}

bool hand_serial::set_angle(const double angle[6]) {
    ROS_DEBUG("hand: set angle");
    double encoder[6];
    for(int i = 0; i < 6; i++) {
        encoder[i] = (1000.0 - 1000.0 * angle[i] / (angle_upper_limit[i] - angle_lower_limit[i]));
    }
    if(validate_values(encoder, -1, 1000)) {
        return set_reg(encoder, 1486); // 返回成功
    }
    return false; // 返回失败
}

bool hand_serial::set_force(const double force[6]) {
    ROS_DEBUG("hand: set force");

    if(validate_values(force, 0, 3000)) {
        return set_reg(force, 1498); // 返回成功
    }
    return false; // 返回失败
}

bool hand_serial::set_default_force(const double force[6]) {
    ROS_DEBUG("Hand: Set Default Force request received");

    if(validate_values(force, 0, 3000) && set_reg(force, 1044)) {
        return save_setting(); // 返回成功
    }
    return false; // 返回失败
}

bool hand_serial::set_force_calibration() {
    ROS_DEBUG("Hand: Set Force Calibration request received");

    uint16_t calibration_value = 1000; // 要写入的校准值

    // 写入寄存器 1486 到 1496
    for (int register_address = 1486; register_address <= 1496; register_address += 2) {
        int rc = writeRegister(register_address, calibration_value);
        if (rc == -1) {
            ROS_ERROR("Failed to write to Modbus register %d: %s", register_address, modbus_strerror(errno));
            return false; // 返回失败
        }
    }

    // 延时 10 毫秒
    usleep(10000); // 10ms

    // 向寄存器 1009 写入 1，进行力控校准
    uint16_t force_calibration_value = 1;
    int rc = writeRegister(1009, force_calibration_value);
    
    if (rc == -1) {
        ROS_ERROR("Failed to write to Modbus register 1009: %s", modbus_strerror(errno));
        return false; // 返回失败
    }
    
    return true; // 返回成功
}

bool hand_serial::set_current_limit(const double current_limit[6]) {
    ROS_DEBUG("Hand: Set Current Limit request received");

    if(validate_values(current_limit, 0, 1500)) {
        return set_reg(current_limit, 1020); // 返回成功
    }
    return false; // 返回失败
}

bool hand_serial::set_clear_error() {
    ROS_DEBUG("Hand: Set CLEAR ERROR request received");

    uint16_t value = 1; // 写入1，代表清除错误
    int rc = writeRegister(1004, value);
    
    if (rc == -1) {
        ROS_ERROR("Failed to write to Modbus register: %s", modbus_strerror(errno));
        return false;
    }
    return true; // 返回成功
}

// Callback to reset parameters
bool hand_serial::set_reset_parameters() {
    ROS_DEBUG("Hand: Set RESET PARAMETER request received");

    uint16_t value = 1; // 写入1，代表重置参数
    int rc = writeRegister(1006, value);
    
    if (rc == -1) {
        ROS_ERROR("Failed to write to Modbus register: %s", modbus_strerror(errno));
        return false; // 返回失败
    }
    return true; // 返回成功
}

std::vector<std::vector<uint16_t>> hand_serial::resize_tactile_data(uint16_t *v, int rows, int cols) {
    // Create a 2D vector
    std::vector<std::vector<uint16_t>> matrix(rows, std::vector<uint16_t>(cols));

    // Fill the matrix from the original vector
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            matrix[i][j] = v[i * cols + j];
        }
    }

    return matrix;
}

std::vector<std::tuple<int, int, int, int, int, bool, std::string>> tactile_read_lookup = {
    {3000, 3, 3, 0, 0, false, "little_finger_tip"},
    {3018, 12, 8, 0, 1, false, "little_finger_nail"},
    {3210, 10, 8, 0, 2, false, "little_finger_pad"},
    {3370, 3, 3, 1, 0, false, "ring_finger_tip"},
    {3388, 12, 8, 1, 1, false, "ring_finger_nail"},
    {3580, 10, 8, 1, 2, false, "ring_finger_pad"},
    {3740, 3, 3, 2, 0, false, "middle_finger_tip"},
    {3758, 12, 8, 2, 1, false, "middle_finger_nail"},
    {3950, 10, 8, 2, 2, false, "middle_finger_pad"},
    {4110, 3, 3, 3, 0, false, "index_finger_tip"},
    {4128, 12, 8, 3, 1, false, "index_finger_nail"},
    {4320, 10, 8, 3, 2, false, "index_finger_pad"},
    {4480, 3, 3, 4, 0, false, "thumb_tip"},
    {4498, 12, 8, 4, 1, false, "thumb_nail"},
    {4690, 3, 3, 4, 2, false, "thumb_middle_section"},
    {4708, 12, 8, 4, 3, true, "thumb_pad"},
    {4900, 8, 14, 0, 4, true, "palm"}
};

cv::Mat hand_serial::convert_tactile_data_to_image(const std::vector<std::vector<std::vector<uint16_t>>>& multi_tactile_data, int rows, int cols) {
    std::vector<cv::Mat> images;

    for (int ind = 0; ind < multi_tactile_data.size() - 1; ind++) {

        int row = multi_tactile_data[ind].size();
        int col = multi_tactile_data[ind][0].size();
        int sub_image_row = std::get<3>(tactile_read_lookup[ind]);
        int sub_image_col = std::get<4>(tactile_read_lookup[ind]);

        cv::Mat image(row, col, CV_8UC1); // Grayscale image (CV_8UC1)
        for (int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                image.at<uchar>(i, j) = multi_tactile_data[ind][i][j] / 16;
            }
        }

        if(std::get<5>(tactile_read_lookup[ind])) {
            cv::rotate(image, image, cv::ROTATE_180);
        }

        if (sub_image_col == 0) {
            images.push_back(image); // Push first image to the vector
        } else {
            // Resize the image and horizontally concatenate with the previous image
            int max_width = std::max(images[sub_image_row].cols, image.cols);
            double scale_ratio = static_cast<double>(max_width) / image.cols;
            cv::Mat resized_image;
            cv::resize(image, resized_image, cv::Size(max_width, static_cast<int>(scale_ratio * image.rows)), 0, 0, cv::INTER_NEAREST);

            scale_ratio = static_cast<double>(max_width) / images[sub_image_row].cols;
            cv::Mat resized_previous_image;
            cv::resize(images[sub_image_row], resized_previous_image, cv::Size(max_width, static_cast<int>(scale_ratio * images[sub_image_row].rows)), 0, 0, cv::INTER_NEAREST);

            // Horizontally concatenate the images
            cv::Mat concatenated_image;
            cv::vconcat(resized_previous_image, resized_image, concatenated_image);

            // Store the result back into the vector
            images[sub_image_row] = concatenated_image;
        }
    }

    // Ensure all images have the same number of rows
    int max_height = images[0].rows;
    for (int i = 1; i < images.size(); i++) {
        max_height = std::max(images[i].rows, max_height);
    }

    for (int i = 0; i < images.size(); i++) {
        if (images[i].rows != max_height) {
            cv::resize(images[i], images[i], cv::Size(images[i].cols, max_height));
        }
    }

    // Vertically concatenate the images
    cv::Mat combined_image = images[0];
    for (int i = 1; i < images.size(); i++) {
        cv::hconcat(combined_image, images[i], combined_image);
    }

    // Resize the final combined image to the desired size
    int row = multi_tactile_data.back().size();
    int col = multi_tactile_data.back()[0].size();
    cv::Mat final_image(row, col, CV_8UC1);
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            final_image.at<uchar>(i, j) = multi_tactile_data.back()[i][j] / 16;
        }
    }
    cv::rotate(final_image, final_image, cv::ROTATE_90_COUNTERCLOCKWISE);
    if(std::get<5>(tactile_read_lookup.back())) {
        cv::flip(final_image, final_image, 0);
    }

    int max_width = std::max(combined_image.cols, final_image.cols);
    double scale_ratio = static_cast<double>(max_width) / final_image.cols;
    cv::resize(final_image, final_image, cv::Size(max_width, static_cast<int>(scale_ratio * final_image.rows)), 0, 0, cv::INTER_NEAREST);

    scale_ratio = static_cast<double>(max_width) / combined_image.cols;
    cv::resize(combined_image, combined_image, cv::Size(max_width, static_cast<int>(scale_ratio * combined_image.rows)), 0, 0, cv::INTER_NEAREST);

    // Horizontally concatenate the final image with the combined image
    cv::vconcat(combined_image, final_image, combined_image);

    // Resize the final combined image to the desired size
    cv::resize(combined_image, combined_image, cv::Size(cols, rows), 0, 0, cv::INTER_NEAREST);

    return combined_image;


}

bool hand_serial::get_tactile_data() {
    std::vector<std::vector<std::vector<uint16_t>>> multi_tactile_data;
    for(int i = 0; i < tactile_read_lookup.size() - 1; i++) {
        uint16_t tactile_data[std::get<1>(tactile_read_lookup[i]) * std::get<2>(tactile_read_lookup[i])];
        if(readRegisters(std::get<0>(tactile_read_lookup[i]), std::get<1>(tactile_read_lookup[i]) * std::get<2>(tactile_read_lookup[i]), tactile_data) == -1) {
            return false;
        }
        multi_tactile_data.push_back(resize_tactile_data(tactile_data, std::get<1>(tactile_read_lookup[i]), std::get<2>(tactile_read_lookup[i])));
    }
    uint16_t tactile_data[std::get<1>(tactile_read_lookup.back()) * std::get<2>(tactile_read_lookup.back())];
    if(readRegisters(std::get<0>(tactile_read_lookup.back()), std::get<1>(tactile_read_lookup.back()) * std::get<2>(tactile_read_lookup.back()), tactile_data) == -1) {
        return false;
    }
    multi_tactile_data.push_back(resize_tactile_data(tactile_data, std::get<2>(tactile_read_lookup.back()), std::get<1>(tactile_read_lookup.back())));
    multi_tactile_data_ = multi_tactile_data;
    multi_tactile_image_ = convert_tactile_data_to_image(multi_tactile_data);
    return true;
}

// Read a register
int hand_serial::readRegister(int reg_addr) {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    uint16_t value;
    if (modbus_read_registers(ctx_, reg_addr, 1, &value) == -1) { // 只读取一个寄存器
        ROS_ERROR("Failed to read register %d: %s", reg_addr, modbus_strerror(errno));
        return -1; // Error
    }
    return value; // 返回读取的值
}

int hand_serial::readRegisters(int reg_addr, int num_registers, uint16_t *tab_reg) {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    if (modbus_read_registers(ctx_, reg_addr, num_registers, tab_reg) == -1) {
        ROS_ERROR("Failed to read registers starting at %d: %s", reg_addr, modbus_strerror(errno));
        return -1; // Error
    }
    return 0; // Success
}

// Write to a register
int hand_serial::writeRegister(int reg_addr, int value) {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    if (modbus_write_register(ctx_, reg_addr, value) == -1) {
        ROS_ERROR("Failed to write register %d: %s", reg_addr, modbus_strerror(errno));
        return -1; // Error
    }
    return 0; // Success
}

// Write multiple registers (optional, if needed)
int hand_serial::writeMultipleRegisters(int start_addr, const uint16_t *values, int num_values) {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    if (modbus_write_registers(ctx_, start_addr, num_values, values) == -1) {
        ROS_ERROR("Failed to write registers starting at %d: %s", start_addr, modbus_strerror(errno));
        return -1; // Error
    }
    return 0; // Success
}
}
#endif
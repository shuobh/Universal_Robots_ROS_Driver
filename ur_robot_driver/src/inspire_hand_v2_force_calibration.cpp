
#include <inspire_hand/hand_control_lib_v2.h>

// write main function to init inspire hand
int main(int argc, char** argv) {
    ros::init(argc, argv, "inspire_hand_force_calibration");
    ros::NodeHandle nh;

    inspire_hand::hand_serial hand;
    // ID of the hand controller. This is used to identify the hand controller in the driver.
    int hand_id = nh.param("hand_id", 1);

    // IP that will be used for the hand controller to communicate back to the driver.
    std::string hand_ip = nh.param<std::string>("hand_ip", "192.168.11.210");

    // Port that will be opened to communicate between the driver and the hand controller.
    int hand_port = nh.param("hand_port", 6000);

    hand.initialize(hand_id, hand_ip, hand_port);

    // Start force calibration
    ROS_INFO("Starting force calibration");
    ROS_INFO("1. Hold five fingers fully open");
    ROS_INFO("2. Bend four fingers (little finger, ring finger, middle finger, and index finger)");
    ROS_INFO("3. Hold the four fingers open, and bend the thumb");
    ROS_INFO("4. Extend the thumb");

    hand.set_force_calibration();

    return 0;
}

#include <inspire_hand/hand_control_lib.h>

// write main function to init inspire hand
int main(int argc, char** argv) {
    ros::init(argc, argv, "inspire_hand_force_calibration");
    ros::NodeHandle nh;

    inspire_hand::hand_serial hand(&nh);

    // Start force calibration
    ROS_INFO("Starting force calibration");
    ROS_INFO("1. Hold five fingers fully open");
    ROS_INFO("2. Bend four fingers (little finger, ring finger, middle finger, and index finger)");
    ROS_INFO("3. Hold the four fingers open, and bend the thumb");
    ROS_INFO("4. Extend the thumb");

    hand.set_force_calibration();

    return 0;
}
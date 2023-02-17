/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

PACKAGE: 	slam
AUTHOR(S):	Max Tory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Include class header
#include "slam_node.h"

const std::chrono::milliseconds POSE_PUB_TIME = 50ms;

// Use the standard namespaces
using std::placeholders::_1;

// Constructor for slam Node class
SLAM::SLAM () : Node("camsub") {
    rclcpp::QoS qos = rclcpp::QoS(5).reliable();

    this->create_subscription<sensor_msgs::msg::Imu>(
        "/T265/imu",
           10, 
        std::bind(&SLAM::imu_callback, this, _1)
    );
    std::cout << "created imu sub" << std::endl;

    // left camera subscription
    this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw",
		10,
        std::bind(&SLAM::image_callback, this, _1)
    );
    std::cout << "created img sub" << std::endl;

    // publisher timer
    this->create_wall_timer(
        POSE_PUB_TIME, 
        std::bind(&SLAM::pose_pub_callback, this)
    );
    std::cout << "created pose timer" << std::endl;

    // pose publisher
    publisher_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/slam/pose", 
        10
    );
    std::cout << "created pose publisher" << std::endl;

    latest_pose = geometry_msgs::msg::PoseStamped();

    // Initialise buffers and mutexes
    img_buf = std::queue<sensor_msgs::msg::Image>();
    imu_buf = std::queue<sensor_msgs::msg::Imu>();
    
    std::cout << "FINISHED CONSTRUCTOR" << std::endl;
}

// Handle IMU message
void SLAM::imu_callback (const sensor_msgs::msg::Imu::SharedPtr msg) {

    std::cout << "IMU CALLBACK" << std::endl;
    // Mutex prevents other threads from accessing the memory while we use it
    imu_buf.push(*msg);
}

// Handle Image message from left lens
void SLAM::image_callback (const sensor_msgs::msg::Image::SharedPtr msg) {

    std::cout << "IMAGE CALLBACK" << std::endl;
    img_buf.push(*msg);
}

// Publish Pose
void SLAM::pose_pub_callback (){
    std::cout << "POSE PUB CALLBACK" << std::endl;
    publisher_pose->publish(latest_pose);
}

//  Main function called when the script execution begins
int main(int argc, char **argv)
{
    // Initialises the ROS C++ class
    rclcpp::init();

    std::cout << "Starting constructor" << std::endl;
    // Runs the Publisher class
    rclcpp::spin(std::make_shared<SLAM>());
    std::cout << "Finished spinning" << std::endl;


    // Shutsdown ROS once complete
    rclcpp::shutdown();

    // Returns an empty value
    return 0;
}

#pragma once

/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

Code that uses the ORB_SLAM3 package to get the rover's
    pose with optional IMU data. Subscribes to IMU and
    camera feeds and expects a command line argument 
    providing a calibration file and ORBSLAM vocabulary

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
NODE: orbslam
TOPICS:
  - imu
  - t265 left
  - t265 right
  - /slam/pose                   [geometry_msgs/PoseStamped] [Published]
SERVICES: None
ACTIONS:  None
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PACKAGE: 	slam
AUTHOR(S):  Max Tory
CREATION:	11/02/2023
EDITED:		11/02/2023
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TODO:
 - 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Include ROS client library
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/time.hpp>
// Include message types
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Standard libraries
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>

using namespace std::chrono_literals;
/* 
Class which receives the commands for the CMDs and drives the joints
*/
class SLAM : public rclcpp::Node
{
    //------------------------------------------------------------//
    private:

    // Timer for publishing our pose estimate
    rclcpp::TimerBase::SharedPtr timer_publish_pose;

    // Publisher for pose estimate
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose;

    // Latest SLAM-estimated pose
    geometry_msgs::msg::PoseStamped latest_pose;

    // Queue of images received from the left and right lens
    std::queue<sensor_msgs::msg::Image> img_buf;

    // Queue of imu messages
    std::queue<sensor_msgs::msg::Imu> imu_buf;

    /// @brief      Callback function when IMU messages are received.
    /// @param      msg - A pointer to the input message
    void imu_callback (const sensor_msgs::msg::Imu::SharedPtr msg);

    /// @brief      Callback function when T265 camera left lens images are received.
    /// @param      msg - A pointer to the input message
    void image_callback (const sensor_msgs::msg::Image::SharedPtr msg);

    /// @brief      Callback function for publishing pose from SLAM
    void pose_pub_callback ();

    //------------------------------------------------------------//
    public:

    /// @brief      Constructor. Starts publishers, subscribers and initialises members
    SLAM();
};

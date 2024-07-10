#include "rclcpp/rclcpp.hpp"
/// include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <vector>
#include <algorithm>
#include <cmath>


class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// create ROS subscribers and publishers
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, std::placeholders::_1));
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Safety Node has been started");
    }

private:
    // initiate some global values
    double speed = 0.0;

    // double time_prev = 0.0;
    // std::vector<float> ranges_prev;

    /// create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// update current speed
        speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// Calculate TTC
        // Get the array of scan angles
        float angle_min = scan_msg->angle_min;
        // float angle_max = scan_msg->angle_max;
        float angle_increment = scan_msg->angle_increment;
        int num_ranges = scan_msg->ranges.size();
        
        std::vector<float> scan_angles(num_ranges);
        for (int i = 0; i < num_ranges; ++i) 
        {
            scan_angles[i] = angle_min + i * angle_increment;
        }

        // Get the array of range rates
        std::vector<float> range_rates(num_ranges);
        for (int i = 0; i < num_ranges; ++i) 
        {
            range_rates[i] = std::max(speed * std::cos(scan_angles[i]), 0.0);
        }

        // Get the TTC array
        std::vector<float> TTC_array(num_ranges);
        for (int i = 0; i < num_ranges; ++i) {
            if (std::isnan(scan_msg->ranges[i])) {
                TTC_array[i] = std::numeric_limits<float>::infinity();
            } else {
                TTC_array[i] = (range_rates[i] != 0) ? scan_msg->ranges[i] / range_rates[i] : std::numeric_limits<float>::infinity();
            }
        }

        /// publish drive/brake message
        float min_value = TTC_array[0];
        int min_index = 0;

        for (int i = 1; i < num_ranges; i++) {
            if (TTC_array[i] < min_value) {
                min_value = TTC_array[i];
                min_index = i;
            }
        }

        if (min_value < 1.3)
        {
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.speed = 0.0;
            drive_publisher_ ->publish(drive_msg);
            float AEB_angle = (angle_min + min_index * (angle_increment)) * (180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), "AEB activated with TTC:  %f at the angle %f", min_value, AEB_angle);
        }
    }
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
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

    double time_prev = 0.0;
    std::vector<float> ranges_prev;

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
        const std::vector<float>& ranges_now = scan_msg->ranges;
        int size = sizeof(ranges_now) / sizeof(ranges_now[0]);
        // std::cout << "size of ranges_now: "<< ranges_now << std::endl;
        // handle the time diff
        int sec = scan_msg->header.stamp.sec;
        // std::cout << "sec: "<< sec << std::endl;
        int nsec = scan_msg->header.stamp.nanosec; // nano second
        // std::cout << "nanosec: "<< nsec << std::endl;
        double time_now = static_cast<double>(sec) + (static_cast<double>(nsec) / 1e9);
        // std::cout << "time_now: "<< time_now << std::endl;
        // std::cout << "time_prev: "<< time_prev << std::endl;
        float dt = time_now - time_prev;
        // std::cout << "Error dt: " << dt << std::endl;
        if (dt <= 0.0) 
        {
            std::cerr << "Error: tiemstamp is incorrect!" << std::endl;
        }

        if (time_prev!=0)
        {
            /// calculate TTC
            std::vector<float> range_rates(size);
            std::transform(ranges_now.begin(),ranges_now.end(),ranges_prev.begin(),range_rates.begin(),
                [dt](float rn, float rp)
                {
                    double rate = -(rn - rp) / dt;
                    if (std::isnan(rn) || std::isinf(rn) || std::isnan(rp) || std::isinf(rp) || rate < 0) 
                    {
                        return 0.0;
                    } else { return rate; }
                });
            std::vector<float> TTC_array(size);
            for (int i = 0; i < size; i++) {
                if (range_rates[i] != 0) {
                    TTC_array[i] = ranges_now[i] / range_rates[i];
                } else {
                    TTC_array[i] = INFINITY; // Use INFINITY macro from cmath library
                }
            }

            /// publish drive/brake message
            float min_value = TTC_array[0];
            int min_index = 0;

            for (int i = 1; i < size; i++) {
                if (TTC_array[i] < min_value) {
                    min_value = TTC_array[i];
                    min_index = i;
                }
            }

            if (min_value < 1.0)
            {
                auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
                drive_msg.drive.speed = 0.0;
                drive_publisher_ ->publish(drive_msg);
                float AEB_angle = (scan_msg->angle_min + min_index * (scan_msg->angle_increment)) * (180.0 / M_PI);
                RCLCPP_INFO(this->get_logger(), "AEB activated with TTC:  %f at the angle %f", min_value, AEB_angle);
            }
        }
        ranges_prev = ranges_now;
        time_prev = time_now;
    }




};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
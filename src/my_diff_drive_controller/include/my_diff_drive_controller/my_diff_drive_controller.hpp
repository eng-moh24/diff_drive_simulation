#pragma once

#include "differential_drive_kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <string>


namespace MyCustomController
{
    class MyDifferentialDriveController : public controller_interface::ControllerInterface
    {
        public:
        controller_interface::CallbackReturn on_init() override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
        controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period ) override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        private:
        std::string left_wheel_joint_ , right_wheel_joint_;
        DiffDriveVehicle my_robot_;

        realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>::SharedPtr rt_odom_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr non_rt_odom_publisher_;

        realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>::SharedPtr rt_odom_tf_publisher_;
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr non_rt_odom_tf_publisher_;
        
        realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> cmd_vel_buf_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    };
}
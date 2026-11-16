#pragma once

#include <math.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

using RobotPose = geometry_msgs::msg::Pose;
using CommandVelocity = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;

class DiffDriveVehicle
{
    public:
    DiffDriveVehicle() : total_measured_velocity_(0.0) ,measured_angular_velocity_(0.0), theta_(0.0),
                        left_wheel_velocity_(0.0) , right_wheel_velocity_(0.0),
                        wheel_separation_(1.0) , wheel_radius_(1.0) {} 

    void calculateForwardKinematics(double left_encoder_velocity , double right_encoder_velocity)
    {
        total_measured_velocity_ = (left_encoder_velocity + right_encoder_velocity) / 2;
        measured_angular_velocity_ = ( left_encoder_velocity - right_encoder_velocity ) / wheel_separation_; 
    }

    void calculateInverseKinematics(CommandVelocity cmd_vel)
    {
        left_wheel_velocity_ = (cmd_vel.linear.x - cmd_vel.angular.z * wheel_separation_ / 2) / wheel_radius_;
        right_wheel_velocity_ = (cmd_vel.linear.x + cmd_vel.angular.z * wheel_separation_ / 2) / wheel_radius_;
    }

    double get_left_wheel_velocity(){ return left_wheel_velocity_;}
    double get_right_wheel_velocity() {return right_wheel_velocity_;}

    Odometry getOdometry (const double dt)
    {
        robot_odometry_.twist.twist.linear.x = total_measured_velocity_;
        robot_odometry_.twist.twist.angular.z = measured_angular_velocity_;
        double theta_midpoint =  theta_ + (measured_angular_velocity_ * dt / 2);

        robot_odometry_.pose.pose.position.x += total_measured_velocity_ * std::cos(theta_midpoint) * dt;
        robot_odometry_.pose.pose.position.y += total_measured_velocity_ * std::sin(theta_midpoint) * dt;

        theta_ += measured_angular_velocity_ * dt;
        robot_odometry_.pose.pose.orientation.z = std::sin(theta_ / 2);
        robot_odometry_.pose.pose.orientation.w = std::cos( theta_ / 2);

        return robot_odometry_;
    }
    void resetOdometry() { robot_odometry_ = nav_msgs::msg::Odometry(); }

    void set_wheel_separation(double wheel_separation) { wheel_separation_ = wheel_separation;}
    void set_wheel_radius(double wheel_radius) { wheel_radius_ = wheel_radius;}

    private:
    double total_measured_velocity_ , measured_angular_velocity_ , theta_;
    double left_wheel_velocity_ , right_wheel_velocity_;
    double wheel_separation_ , wheel_radius_;
    Odometry robot_odometry_;
};

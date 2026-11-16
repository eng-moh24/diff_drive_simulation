#include "my_diff_drive_controller/my_diff_drive_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

using namespace MyCustomController;

controller_interface::CallbackReturn MyDifferentialDriveController::on_init()
{
    node_ = get_node();
    try
    {
        auto_declare("left_wheel_joint" , left_wheel_joint_);
        auto_declare("right_wheel_joint" , right_wheel_joint_);
        auto_declare("wheel_separation" , 1.0);
        auto_declare("wheel_radius" , 1.0);
        auto_declare("odom_frame_id", std::string("odom"));
        auto_declare("base_frame_id", std::string("base_link"));
        auto_declare("publish_tf", true);
    }
    catch(const std::exception& e)
    {
       RCLCPP_ERROR( node_->get_logger() , "Error on controller %s at inital phase: %s" , 
       node_->get_name() , e.what());
       return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MyDifferentialDriveController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = { left_wheel_joint_ + "/velocity" , right_wheel_joint_ + "/velocity"};
    return config;
}

controller_interface::InterfaceConfiguration MyDifferentialDriveController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = { left_wheel_joint_ + "/velocity" , right_wheel_joint_ + "/velocity"};
    return config;
}

controller_interface::CallbackReturn MyDifferentialDriveController::on_configure(const rclcpp_lifecycle::State &)
{
    try
    {
        my_robot_.set_wheel_radius(node_->get_parameter("wheel_radius").as_double());
        my_robot_.set_wheel_separation(node_->get_parameter("wheel_separation").as_double());
        left_wheel_joint_ = node_->get_parameter("left_wheel_joint").as_string();
        right_wheel_joint_ = node_->get_parameter("right_wheel_joint").as_string();

        cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel" , 10 , [this](geometry_msgs::msg::Twist::SharedPtr msg){
            cmd_vel_buf_.writeFromNonRT(*msg);
        });
        non_rt_odom_publisher_ = rclcpp::create_publisher<nav_msgs::msg::Odometry>(node_ , "odom" , rclcpp::SystemDefaultsQoS());
        non_rt_odom_tf_publisher_ = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(node_ , "tf" , rclcpp::SystemDefaultsQoS());

        rt_odom_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(non_rt_odom_publisher_);
        rt_odom_publisher_->msg_.header.frame_id = node_->get_parameter("odom_frame_id").as_string();
        rt_odom_publisher_->msg_.child_frame_id = node_->get_parameter("base_frame_id").as_string();

        rt_odom_tf_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(non_rt_odom_tf_publisher_);
        rt_odom_tf_publisher_->msg_.transforms.resize(1);
        rt_odom_tf_publisher_->msg_.transforms[0].header.frame_id = node_->get_parameter("odom_frame_id").as_string();
        rt_odom_tf_publisher_->msg_.transforms[0].child_frame_id = node_->get_parameter("base_frame_id").as_string();
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR( node_->get_logger() , "Error on controller %s at configuration phase: %s" ,
        node_->get_name() , e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyDifferentialDriveController::on_activate(const rclcpp_lifecycle::State &)
{
    cmd_vel_buf_.writeFromNonRT(geometry_msgs::msg::Twist());
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyDifferentialDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
    for ( auto& ci : command_interfaces_ ) { ci.set_value(0.0);}
    return CallbackReturn::SUCCESS;
}


controller_interface::return_type MyDifferentialDriveController::update(const rclcpp::Time & time , const rclcpp::Duration & period)
{
    if (period.seconds() <= 0.0){
        return controller_interface::return_type::OK;
    }

    auto cmd_vel = *cmd_vel_buf_.readFromRT();
    my_robot_.calculateInverseKinematics(cmd_vel);

    command_interfaces_[0].set_value(my_robot_.get_left_wheel_velocity());
    command_interfaces_[1].set_value(my_robot_.get_right_wheel_velocity()); 

    my_robot_.calculateForwardKinematics(state_interfaces_[0].get_value() , state_interfaces_[1].get_value());
    auto Odometry = my_robot_.getOdometry(period.seconds());

    if ( rt_odom_publisher_->trylock()){
        rt_odom_publisher_->msg_.pose.pose.position.x = Odometry.pose.pose.position.x;
        rt_odom_publisher_->msg_.pose.pose.position.y = Odometry.pose.pose.position.y;
        rt_odom_publisher_->msg_.pose.pose.orientation.z = Odometry.pose.pose.orientation.z;
        rt_odom_publisher_->msg_.pose.pose.orientation.w = Odometry.pose.pose.orientation.w;
        rt_odom_publisher_->unlockAndPublish();
    }
    if (rt_odom_tf_publisher_->trylock()){
        rt_odom_tf_publisher_->msg_.transforms[0].header.stamp = time;
        rt_odom_tf_publisher_->msg_.transforms[0].transform.translation.x = Odometry.pose.pose.position.x;
        rt_odom_tf_publisher_->msg_.transforms[0].transform.translation.y = Odometry.pose.pose.position.y;
        rt_odom_tf_publisher_->msg_.transforms[0].transform.rotation.z = Odometry.pose.pose.orientation.z;
        rt_odom_tf_publisher_->msg_.transforms[0].transform.rotation.w = Odometry.pose.pose.orientation.w;
        rt_odom_tf_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS( MyCustomController::MyDifferentialDriveController , controller_interface::ControllerInterface);

#include "controller/my_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace MyController
{
    controller_interface::CallbackReturn ForwardVelocityController::on_init()
    {
        try
        {
            node_ = get_node();
            auto_declare<std::vector<std::string>>("joints" , {});
            auto_declare<double>("velocity" , 0.0);
            auto_declare<double>("status_rate" , 1.0);
            return CallbackReturn::SUCCESS;
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR( node_->get_logger() , "Failure to initialize: %s" , e.what());
            return CallbackReturn::ERROR;
        }
    }

    controller_interface::CallbackReturn ForwardVelocityController::on_configure(const rclcpp_lifecycle::State &)
    {
        joints_ = node_->get_parameter("joints").as_string_array();
        velocity_ = node_->get_parameter("velocity").as_double();
        status_ = node_->get_parameter("status_rate").as_double();

        if (joints_.empty()) {
            RCLCPP_ERROR( node_->get_logger() , "There's no joints");
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration ForwardVelocityController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for ( auto joint : joints_) { config.names.push_back(joint + "/velocity");}

        return config;
    }

    controller_interface::CallbackReturn ForwardVelocityController::on_activate(const rclcpp_lifecycle::State &)
    {
        for ( auto & ci : command_interfaces_ ) { ci.set_value(0.0);}
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type ForwardVelocityController::update( const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        velocity_ = node_->get_parameter("velocity").as_double();
        for ( auto& ci : command_interfaces_){ ci.set_value(velocity_);}
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn ForwardVelocityController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        for ( auto & ci : command_interfaces_ ) { ci.set_value(0.0);}
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration ForwardVelocityController::state_interface_configuration() const
    {
        return { controller_interface::interface_configuration_type::NONE };
    }
}

PLUGINLIB_EXPORT_CLASS( MyController::ForwardVelocityController , controller_interface::ControllerInterface )


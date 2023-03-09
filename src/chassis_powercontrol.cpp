#include"rclcpp/rclcpp.hpp"
#include "gary_msgs/msg/power_heat.hpp"
#include "gary_msgs/msg/robot_status.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include <chrono>

using namespace std::chrono_literals;

gary_msgs::msg::PowerHeat power;
gary_msgs::msg::RobotStatus power_limit;

class PowerControl : public rclcpp::Node
{
public:
    PowerControl() : Node("power_control")
    {
        power_sub_ = create_subscription<gary_msgs::msg::PowerHeat>("power",rclcpp::SystemDefaultsQoS(),
                                                                    std::bind(&PowerControl::power_callback,this,std::placeholders::_1));
        power_limit_sub = create_subscription<gary_msgs::msg::RobotStatus>("power_limit",rclcpp::SystemDefaultsQoS(),
                                                                           std::bind(&PowerControl::powerlimit_callback,this,std::placeholders::_1));
        timer_ = this->create_wall_timer(500ms,std::bind(&PowerControl::time_callback,this));
        joint_sub = this->create_subscription<control_msgs::msg::DynamicJointState>(
                    "/dynamic_joint_states", rclcpp::SystemDefaultsQoS(),
                std::bind(&PowerControl::joint_callback, this, std::placeholders::_1));
    }
private:
    void powerlimit_callback(gary_msgs::msg::RobotStatus::SharedPtr msg)
    {
        power_limit = *msg;
    }
    void power_callback(gary_msgs::msg::PowerHeat::SharedPtr msg)
    {
        power = *msg;
    }

    void joint_callback(control_msgs::msg::DynamicJointState::SharedPtr joint_state){
        for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
            if(joint_state->joint_names[i] == "chassis_lf")
            {
                for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j)
                {
                    if (joint_state->interface_values[i].interface_names[j] == "effort_raw")
                    {
                        this->lf_current = joint_state->interface_values[i].values[j];
                    }
                }
            }
        }
        for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
            if(joint_state->joint_names[i] == "chassis_lb")
            {
                for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j)
                {
                    if (joint_state->interface_values[i].interface_names[j] == "effort_raw")
                    {
                        this->lb_current= joint_state->interface_values[i].values[j];
                    }
                }
            }
        }
        for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
            if(joint_state->joint_names[i] == "chassis_rf")
            {
                for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j)
                {
                    if (joint_state->interface_values[i].interface_names[j] == "effort_raw")
                    {
                        this->rf_current = joint_state->interface_values[i].values[j];
                    }
                }
            }
        }
        for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
            if(joint_state->joint_names[i] == "chassis_rb")
            {
                for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j)
                {
                    if (joint_state->interface_values[i].interface_names[j] == "effort_raw")
                    {
                        this->rb_current = joint_state->interface_values[i].values[j];
                    }
                }
            }
        }
    }
    void time_callback()
    {
        const float warning_power_buffer = 50.0,warning_power = 40.0;
        const float buffer_total_current_limit = 16000.0 , power_total_current_limit = 20000.0;
        float total_current_limit = 0.0;
        if(power.chassis_power_buffer < warning_power_buffer)
        {
            float power_scale;
            if(power.chassis_power_buffer > 5.0)
            {
                power_scale = power.chassis_power_buffer / warning_power_buffer;
            }
            else{
                power_scale = 5.0f / warning_power_buffer;
            }
            total_current_limit = buffer_total_current_limit * power_scale;
        }
    else{
            if(power.chassis_power > 40.0)//warning_power
            {
                float power_scale;

                if(power.chassis_power < 80.0)//power_limit
                {

                    power_scale = (80.0 - power.chassis_power) / (80.0 - 40.0);

                }
                else{
                    power_scale = 0.0f;
                }

                total_current_limit = buffer_total_current_limit + power_total_current_limit * power_scale;
            }
            else{
                total_current_limit = buffer_total_current_limit + power_total_current_limit;
            }
    }
    float total_curernt = lb_current + lf_current + rb_current + rf_current;
        if(total_curernt > total_current_limit)
        {
            float current_scale = total_current_limit / total_curernt;

        }
    }
    float lb_current;
    float rb_current;
    float rf_current;
    float lf_current;
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_sub;
    rclcpp::Subscription<gary_msgs::msg::PowerHeat>::SharedPtr power_sub_;
    rclcpp::Subscription<gary_msgs::msg::RobotStatus>::SharedPtr power_limit_sub;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PowerControl>());
    rclcpp::shutdown();
    return 0;
}
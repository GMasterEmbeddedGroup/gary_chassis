#include"rclcpp/rclcpp.hpp"
#include "gary_msgs/msg/power_heat.hpp"
#include <chrono>

using namespace std::chrono_literals;

gary_msgs::msg::PowerHeat power;

class PowerControl : public rclcpp::Node
{
public:
    PowerControl() : Node("power_control")
    {
        power_sub_ = create_subscription<gary_msgs::msg::PowerHeat>("power",rclcpp::SystemDefaultsQoS(),
                                                                    std::bind(&PowerControl::power_callback,this,std::placeholders::_1));
        timer_ = this->create_wall_timer(500ms,std::bind(&PowerControl::time_callback,this));
    }
private:
    void power_callback(gary_msgs::msg::PowerHeat::SharedPtr msg)
    {
        power = *msg;
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

    }
    rclcpp::Subscription<gary_msgs::msg::PowerHeat>::SharedPtr power_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PowerControl>());
    rclcpp::shutdown();
    return 0;
}
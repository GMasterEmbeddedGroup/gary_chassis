#include "rclcpp/rclcpp.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>

using namespace std::chrono_literals;

gary_msgs::msg::DR16Receiver RC_control;

class ChassisTask : public rclcpp::Node
{
public:
    ChassisTask() : Node("chassis_task"){

        rc_sub = this->create_subscription<gary_msgs::msg::DR16Receiver>("/remote_control",rclcpp::SystemDefaultsQoS(),std::bind(&ChassisTask::rc_callback,this,std::placeholders::_1));
        pub1_ = this->create_publisher<std_msgs::msg::Float64>("/chassis_rb_pid/cmd",rclcpp::SystemDefaultsQoS());
        pub2_ = this->create_publisher<std_msgs::msg::Float64>("/chassis_rf_pid/cmd",rclcpp::SystemDefaultsQoS());
        pub3_ = this->create_publisher<std_msgs::msg::Float64>("/chassis_lb_pid/cmd",rclcpp::SystemDefaultsQoS());
        pub4_ = this->create_publisher<std_msgs::msg::Float64>("/chassis_lf_pid/cmd",rclcpp::SystemDefaultsQoS());
        timer = this->create_wall_timer(10ms,std::bind(&ChassisTask::time_callback,this));
    }
private:
    void rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg)
    {
        RC_control = *msg;
    }
    void time_callback()
    {
        static float x,y,z;
        if(RC_control.sw_right == RC_control.SW_DOWN)
        {
            x=0;
            y=0;
            z=0;
        }
        else if(RC_control.sw_right == RC_control.SW_MID)
        {
         x = RC_control.ch_left_x/660;
         y = RC_control.ch_left_y/660;
         z = RC_control.ch_right_y/660;
        }
        double rf,rb,lf,lb;
        rf = -x-y+(-0.2*z);
        rb = x-y+(-0.2*z);
        lf = x+y+(-0.2*z);
        lb = -x+y+(-0.2*z);
        std::map<std::string, double> wheel_speed;
        wheel_speed.insert(std::make_pair("left_front", lf));
        wheel_speed.insert(std::make_pair("left_back", lb));
        wheel_speed.insert(std::make_pair("right_front", rf));
        wheel_speed.insert(std::make_pair("right_back", rb));
        std_msgs::msg::Float64 data;
        data.data = wheel_speed["left_front"];
        pub1_->publish(data);
        data.data = wheel_speed["left_back"];
        pub2_->publish(data);
        data.data = wheel_speed["right_front"];
        pub3_->publish(data);
        data.data = wheel_speed["right_back"];
        pub4_->publish(data);

    }
    rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr rc_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub1_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub2_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub3_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub4_;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisTask>());
    return 0;
}
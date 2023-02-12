#include "rclcpp/rclcpp.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

gary_msgs::msg::DR16Receiver RC_control;

class ModeChanger : public rclcpp::Node
{
public:
    ModeChanger() : Node("chassis_task"){
         rc_sub = this->create_subscription<gary_msgs::msg::DR16Receiver>("/remote_control",rclcpp::SystemDefaultsQoS(),
                                                          std::bind(&ModeChanger::rc_callback,this,std::placeholders::_1));
         twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/topic_cmd",rclcpp::SystemDefaultsQoS());
         //timer = this->create_wall_timer(100ms,std::bind(&ModeChanger::time_callback,this));
    }
private:
    void rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg)
    {
        RC_control = *msg;
        geometry_msgs::msg::Twist twist;
        if(RC_control.sw_right == RC_control.SW_DOWN)
        {
            return;
        }
        else if(RC_control.sw_right == RC_control.SW_MID)
        {
        twist.linear.x = RC_control.ch_left_x/660 * 50;
        twist.linear.y = RC_control.ch_left_y/660 * 50;
        twist.angular.z = RC_control.ch_right_x/660 * 50;
        }
        std::map<std::string,double> chassis_speed;
        chassis_speed.insert(std::make_pair("vx",twist.linear.x));
        chassis_speed.insert(std::make_pair("vy",twist.linear.y));
        chassis_speed.insert(std::make_pair("az",twist.angular.z));

        geometry_msgs::msg::Twist data;
        twist_pub -> publish(data);
        
    }
    /*void time_callback()
    {

    }*/
    rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr rc_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
    //rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeChanger>());
    return 0;
}
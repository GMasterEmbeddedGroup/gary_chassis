#include"rclcpp/rclcpp.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <chrono>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#include "tf2/convert.h"
#include "utils/mecanum_kinematics.hpp"

using namespace std::chrono_literals;

using namespace gary_chassis;


class ODOM : public rclcpp::Node
{
public:
    ODOM() : Node("Odometry")
    {
        joint_sub = this->create_subscription<control_msgs::msg::DynamicJointState>(
                "/dynamic_joint_states", rclcpp::SystemDefaultsQoS(),
                std::bind(&ODOM::joint_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(500ms,std::bind(&ODOM::time_callback,this));
        odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom",
                                                                               rclcpp::SystemDefaultsQoS());
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/gimbal_imu_broadcaster/imu",rclcpp::SystemDefaultsQoS(),
                                                    std::bind(&ODOM::imu_callback,this,std::placeholders::_1));

    }
private:
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
        Imu = *msg;
        tf2::Quaternion imu_quaternion(Imu.orientation.x,Imu.orientation.y,Imu.orientation.z,Imu.orientation.w);
        tf2::Matrix3x3 m(imu_quaternion);
    }

    void joint_callback(control_msgs::msg::DynamicJointState::SharedPtr joint_state) {
        for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
            if(joint_state->joint_names[i] == "chassis_lf")
            {
                for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j)
                {
                    if (joint_state->interface_values[i].interface_names[j] == "velocity")
                    {
                        this->lf_speed = joint_state->interface_values[i].values[j];
                    }
                }
            }
        }
        for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
            if(joint_state->joint_names[i] == "chassis_lb")
            {
                for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j)
                {
                    if (joint_state->interface_values[i].interface_names[j] == "velocity")
                    {
                        this->lb_speed = joint_state->interface_values[i].values[j];
                    }
                }
            }
        }
        for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
            if(joint_state->joint_names[i] == "chassis_rf")
            {
                for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j)
                {
                    if (joint_state->interface_values[i].interface_names[j] == "velocity")
                    {
                        this->rf_speed = joint_state->interface_values[i].values[j];
                    }
                }
            }
        }
        for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
            if(joint_state->joint_names[i] == "chassis_rb")
            {
                for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j)
                {
                    if (joint_state->interface_values[i].interface_names[j] == "velocity")
                    {
                        this->rb_speed = joint_state->interface_values[i].values[j];
                    }
                }
            }
        }
    }
    void time_callback(){
        std::map<std::string, double> chassis_speed_odom;
        std::map<std::string, double> wheel_speed_odom;

        mecanum_kinematics = std::make_shared<gary_chassis::MecanumKinematics>(this->a, this->b, this->r);
        wheel_speed_odom.emplace("left_back",lb_speed);
        wheel_speed_odom.emplace("right_back",rb_speed);
        wheel_speed_odom.emplace("left_front",lf_speed);
        wheel_speed_odom.emplace("right_front",rf_speed);
        chassis_speed_odom = mecanum_kinematics->forward_solve(wheel_speed_odom,WHEEL_OFFLINE_NONE);


    }
    double lb_speed;
    double lf_speed;
    double rb_speed;
    double rf_speed;
    double a = 0.185;
    double b = 0.18922;
    double r = 0.075;
    double x_base_link = 0;
    double y_base_link = 0;
    double z_base_link = 0;

    nav_msgs::msg::Odometry send_Od_data;
    sensor_msgs::msg::Imu Imu;
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<gary_chassis::MecanumKinematics> mecanum_kinematics;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ODOM>());
    rclcpp::shutdown();
    return 0;
}
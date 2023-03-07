#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "diagnostic_msgs//msg/diagnostic_array.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "utils/mecanum_kinematics.hpp"
#include "std_msgs/msg/float64.hpp"
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_chassis {

    class OdometryR : public rclcpp_lifecycle::LifecycleNode{
    public:
        explicit OdometryR(const rclcpp::NodeOptions & options);

    private:
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg);
        void joint_callback(control_msgs::msg::DynamicJointState::SharedPtr joint_state);
        void time_callback();
        std::string odom_topic;
        std::string twist_topic;
        std::string joint_topic;
        std::string time;

        double lb_speed;
        double lf_speed;
        double rb_speed;
        double rf_speed;
        double a;
        double b;
        double r;
        geometry_msgs::msg::Twist twist_msg;
        nav_msgs::msg::Odometry odom_msg;
        //subscriber
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber;
        rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_subscriber;
        //publisher
        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;

        std::shared_ptr<gary_chassis::MecanumKinematics> mecanum_kinematics;
        rclcpp::TimerBase::SharedPtr timer;
    };

}
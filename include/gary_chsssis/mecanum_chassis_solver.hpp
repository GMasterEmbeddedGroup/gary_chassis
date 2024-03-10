#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "utils/mecanum_kinematics.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "diagnostic_msgs//msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <string>
#include <cmath>
#include <algorithm>


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_chassis {

    class MecanumChassisSolver : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit MecanumChassisSolver(const rclcpp::NodeOptions & options);

    private:
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        //callback group
        rclcpp::CallbackGroup::SharedPtr cb_group;

        //callbacks
        void cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg);
        void diag_callback(diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
        void joint_callback(control_msgs::msg::DynamicJointState::SharedPtr joint_state);

        //param
        std::string cmd_topic;
        std::string diagnostic_topic;
        std::string odom_topic;
        std::string output_lf_topic;
        std::string output_lb_topic;
        std::string output_rf_topic;
        std::string output_rb_topic;
        std::string motor_lf_hw_id;
        std::string motor_lb_hw_id;
        std::string motor_rf_hw_id;
        std::string motor_rb_hw_id;
        std::string joint_topic;
        double x,y,z;
        double z_angle;
        double vx_o;
        double vy_o;
        double az_o;
        double lb_speed;
        double lf_speed;
        double rb_speed;
        double rf_speed;
        double a;
        double b;
        double r;

        //time log
        rclcpp::Time current_time,last_time;

        //subscriber
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber;
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_subscriber;
        rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_sub;

        //publisher
        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr lf_publisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr lb_publisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr rf_publisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr rb_publisher;

        std::shared_ptr<gary_chassis::MecanumKinematics> mecanum_kinematics;

        diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
    };
}
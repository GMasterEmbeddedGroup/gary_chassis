#include <std_msgs/msg/float64.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "gary_msgs/msg/pid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "utils/first_order_filter.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"

#define PI 3.141592f

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_chassis {
class ChassisTeleop : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit ChassisTeleop(const rclcpp::NodeOptions &options);

private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

    void rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg);
    void joint_callback(control_msgs::msg::DynamicJointState::SharedPtr joint_state);
    void angle_follow_callback(gary_msgs::msg::PID::SharedPtr msg);
    //params
    std::string remote_control_topic;
    std::string angle_set_topic;
    std::string joint_topic;
    std::string cmd_topic;
    std::string angle_follow;
    const double foward_encoder = 5.102787;
    double encoder;

    double foward_position;
    double foward_if;

    double y_max_speed;
    double x_max_speed;
    double rotate_max_speed;
    double x_max_accel;
    double y_max_accel;

    gary_msgs::msg::DR16Receiver RC_control;
    geometry_msgs::msg::Twist twist;
    gary_msgs::msg::PID angle_follow_pid;

    //publisher
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr angle_pid_set_pub;
    //subscriber
    rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr rc_subscriber;
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_subscriber;
    rclcpp::Subscription<gary_msgs::msg::PID>::SharedPtr angle_follow_sub;
    //filter
    std::shared_ptr<gary_chassis::First_orderFilter> x_filter;
    std::shared_ptr<gary_chassis::First_orderFilter> y_filter;
    };

    class Motor{
    public:
        double absolute_angle;
        std_msgs::msg::Float64 sub_angle;
        double absolute_angle_set;
        double absolute_angle_max;
        double absolute_angle_min;
        double relative_angle_pre;
        double relative_angle;

        double ecd_transform;
        double ecd_set;
        double ecd_delta;
        double pid_set;

    };

    Motor yaw;
    Motor pitch;
}

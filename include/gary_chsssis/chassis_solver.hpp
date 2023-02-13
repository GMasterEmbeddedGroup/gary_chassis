#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "utils/mecanum_kinematics.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_chassis {

    class ChassisSolver : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit ChassisSolver(const rclcpp::NodeOptions & options);

    private:
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        void cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg);

        //params
        std::string cmd_topic;
        std::string output_lf_topic;
        std::string output_lb_topic;
        std::string output_rf_topic;
        std::string output_rb_topic;
        double a;
        double b;
        double r;

        //subscriber
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber;

        //publisher
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr lf_publisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr lb_publisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr rf_publisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr rb_publisher;

        std::shared_ptr<gary_chassis::MecanumKinematics> mecanum_kinematics;
    };
}
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gary_msgs/msg/robot_status.hpp"
#include "gary_msgs/msg/power_heat.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>
#include <cmath>
#include <algorithm>


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_chassis {

    class ChassisPowerControl : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit ChassisPowerControl(const rclcpp::NodeOptions &options);

    private:
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

        //callback group
        rclcpp::CallbackGroup::SharedPtr cb_group;

        //callbacks
        void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg);
        void power_heat_callback(gary_msgs::msg::PowerHeat::SharedPtr msg);
        void robot_status_callback(gary_msgs::msg::RobotStatus::SharedPtr msg);
        void update();

        //params
        std::string twist_pub_topic;
        std::string twist_sub_topic;
        std::string power_heat_topic;
        std::string robot_status_topic;
        double buffer_control_level{};
        double buffer_min_level{};

        //publisher
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;

        //subscriber
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber;
        rclcpp::Subscription<gary_msgs::msg::PowerHeat>::SharedPtr power_heat_subscriber;
        rclcpp::Subscription<gary_msgs::msg::RobotStatus>::SharedPtr robot_status_subscriber;

        double max_power{};
        double scale_factor{};
    };
}

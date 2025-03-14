#include <std_msgs/msg/float64.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "gary_msgs/msg/pid.hpp"
#include "gary_msgs/msg/robot_status.hpp"
#include "gary_msgs/msg/game_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "utils/first_order_filter.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_chassis {

    class ChassisAutonomous : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit ChassisAutonomous(const rclcpp::NodeOptions &options);

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
        void rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg);
        void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
        void robot_status_callback(gary_msgs::msg::RobotStatus::SharedPtr msg);
        void game_status_callback(gary_msgs::msg::GameStatus::SharedPtr msg);
        void update();
        void decision();

        //params
        std::string twist_pub_topic;
        std::string remote_control_topic;
        double x_max_speed{};
        double y_max_speed{};
        double rotate_max_speed{};
        double x_max_accel{};
        double y_max_accel{};
        double update_rate{};

        //publisher
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;

        //subscriber
        rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr rc_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
        rclcpp::Subscription<gary_msgs::msg::RobotStatus>::SharedPtr robot_status_subscriber;
        rclcpp::Subscription<gary_msgs::msg::GameStatus>::SharedPtr game_status_subscriber;

        //timer
        rclcpp::TimerBase::SharedPtr timer_update;
        rclcpp::TimerBase::SharedPtr timer_decision;

        //filter
        std::shared_ptr<gary_chassis::First_orderFilter> x_filter;
        std::shared_ptr<gary_chassis::First_orderFilter> y_filter;
        std::shared_ptr<gary_chassis::First_orderFilter> a_filter;

        //msg received and timestamp
        gary_msgs::msg::DR16Receiver rc;
        rclcpp::Time rc_timestamp;
        nav_msgs::msg::Odometry odom;
        rclcpp::Time odom_timestamp;

        rclcpp::Time decision_timestamp;

        double x_set{};
        double y_set{};
        double z_set{};
        bool on_position = false;
        double max_hp{600};
        double remain_hp{600};
        bool game_started = false;
        double error_threshold = 0.1;
        bool custom = false;
    };
}

#include <std_msgs/msg/float64.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "gary_msgs/msg/pid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "utils/first_order_filter.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include <cmath>


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_chassis {

    enum chassis_mode_e {
        CHASSIS_MODE_ZERO_FORCE,
        CHASSIS_MODE_NORMAL,
        CHASSIS_MODE_FOLLOW_GIMBAL,
        CHASSIS_MODE_SPIN,
    };

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

        //callbacks
        void rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg);
        void joint_callback(control_msgs::msg::DynamicJointState::SharedPtr msg);
        void gimbal_follow_callback(gary_msgs::msg::PID::SharedPtr msg);
        void update();

        //params
        std::string twist_pub_topic;
        std::string remote_control_topic;
        std::string joint_topic;
        std::string gimbal_follow_set_topic;
        std::string gimbal_follow_fdb_topic;
        double x_max_speed{};
        double y_max_speed{};
        double rotate_max_speed{};
        double x_max_accel{};
        double y_max_accel{};
        bool use_break{};
        double update_rate{};
        double yaw_encoder_bias{};

        //publisher
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr gimbal_follow_set_publisher;

        //subscriber
        rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr rc_subscriber;
        rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_subscriber;
        rclcpp::Subscription<gary_msgs::msg::PID>::SharedPtr gimbal_follow_sub;

        //timer
        rclcpp::TimerBase::SharedPtr timer_update;

        //filter
        std::shared_ptr<gary_chassis::First_orderFilter> x_filter;
        std::shared_ptr<gary_chassis::First_orderFilter> y_filter;

        //msg received and timestamp
        gary_msgs::msg::DR16Receiver rc;
        rclcpp::Time rc_timestamp;
        control_msgs::msg::DynamicJointState joint_state;
        rclcpp::Time joint_state_timestamp;
        gary_msgs::msg::PID gimbal_follow_pid;
        rclcpp::Time gimbal_follow_pid_timestamp;

        chassis_mode_e chassis_mode{};
        chassis_mode_e last_chassis_mode{};
        uint8_t last_sw_state{};
    };
}

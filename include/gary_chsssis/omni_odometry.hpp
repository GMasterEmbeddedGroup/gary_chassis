#include <std_msgs/msg/float64.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "utils/omni_kinematics.hpp"
#include <cmath>


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_chassis{
    class OmniOdom : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit OmniOdom(const rclcpp::NodeOptions &options);

    private:
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

        //callbacks
        void joint_callback(control_msgs::msg::DynamicJointState::SharedPtr msg);
        void update();

        //params
        std::string joint_topic;
        std::string omni_odom_topic;
        bool use_break{};
        double update_rate{};
        //odom params
        rclcpp::Time omni_current_time,omni_last_time;
        double x,y,z;
        double z_angle;
        double vx_o;
        double vy_o;
        double az_o;
        //kinematics
        double a;
        double b;
        double r;
        std::shared_ptr<gary_chassis::OmniKinematics> omni_kinematics;

        //publisher
        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr omni_odom_publisher;

        //subscriber
        rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_subscriber;

        //timer
        rclcpp::TimerBase::SharedPtr timer_update;

        //msg received and timestamp
        control_msgs::msg::DynamicJointState joint_state;
        rclcpp::Time joint_state_timestamp;

    };
}
#include "gary_chsssis/chassis_power_control.hpp"


using namespace std::chrono_literals;

using namespace gary_chassis;


ChassisPowerControl::ChassisPowerControl(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
        "chassis_power_control", options) {
    //declare params
    this->declare_parameter("twist_pub_topic", "/cmd_vel_limited");
    this->declare_parameter("twist_sub_topic", "/cmd_vel");
    this->declare_parameter("power_heat_topic", "/referee/power_heat");
    this->declare_parameter("robot_status_topic", "/referee/robot_status");
    this->declare_parameter("buffer_control_level", 150.0f);
    this->declare_parameter("buffer_min_level", 50.0f);
}

CallbackReturn ChassisPowerControl::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //create callback group
    this->cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = this->cb_group;

    //get twist_pub_topic
    this->twist_pub_topic = this->get_parameter("twist_pub_topic").as_string();
    this->twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>(this->twist_pub_topic,
                                                                              rclcpp::SystemDefaultsQoS());
    //get twist_sub_topic
    this->twist_sub_topic = this->get_parameter("twist_sub_topic").as_string();
    this->twist_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            this->twist_sub_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisPowerControl::twist_callback, this, std::placeholders::_1), sub_options);

    //get power_heat_topic
    this->power_heat_topic = this->get_parameter("power_heat_topic").as_string();
    this->power_heat_subscriber = this->create_subscription<gary_msgs::msg::PowerHeat>(
            this->power_heat_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisPowerControl::power_heat_callback, this, std::placeholders::_1), sub_options);

    //get robot_status_topic
    this->robot_status_topic = this->get_parameter("robot_status_topic").as_string();
    this->robot_status_subscriber = this->create_subscription<gary_msgs::msg::RobotStatus>(
            this->robot_status_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisPowerControl::robot_status_callback, this, std::placeholders::_1), sub_options);

    //get buffer_control_level
    this->buffer_control_level = this->get_parameter("buffer_control_level").as_double();

    //get buffer_min_level
    this->buffer_min_level = this->get_parameter("buffer_min_level").as_double();

    RCLCPP_INFO(this->get_logger(), "configured");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisPowerControl::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    this->twist_publisher.reset();
    this->twist_subscriber.reset();
    this->power_heat_subscriber.reset();
    this->robot_status_subscriber.reset();

    RCLCPP_INFO(this->get_logger(), "cleaning up");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisPowerControl::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //activate lifecycle publisher
    this->twist_publisher->on_activate();

    RCLCPP_INFO(this->get_logger(), "activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisPowerControl::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //deactivate lifecycle publisher
    this->twist_publisher->on_deactivate();

    RCLCPP_INFO(this->get_logger(), "deactivated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisPowerControl::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->twist_publisher.get() != nullptr) this->twist_publisher.reset();
    if (this->twist_subscriber.get() != nullptr) this->twist_subscriber.reset();
    if (this->power_heat_subscriber.get() != nullptr) this->power_heat_subscriber.reset();
    if (this->robot_status_subscriber.get() != nullptr) this->robot_status_subscriber.reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisPowerControl::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->twist_publisher.get() != nullptr) this->twist_publisher.reset();
    if (this->twist_subscriber.get() != nullptr) this->twist_subscriber.reset();
    if (this->power_heat_subscriber.get() != nullptr) this->power_heat_subscriber.reset();
    if (this->robot_status_subscriber.get() != nullptr) this->robot_status_subscriber.reset();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}


void ChassisPowerControl::twist_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
    geometry_msgs::msg::Twist twist;
    twist.angular.x = msg->angular.x * this->scale_factor;
    twist.angular.y = msg->angular.y * this->scale_factor;
    twist.angular.z = msg->angular.z * this->scale_factor;
    twist.linear.x = msg->linear.x * this->scale_factor;
    twist.linear.y = msg->linear.y * this->scale_factor;
    twist.linear.z = msg->linear.z * this->scale_factor;
    this->twist_publisher->publish(twist);
}


void ChassisPowerControl::power_heat_callback(gary_msgs::msg::PowerHeat::SharedPtr msg) {
    if (static_cast<double>(msg->chassis_power_buffer) > this->buffer_control_level) {
        this->scale_factor = 1.0;
    } else {
        this->scale_factor = (static_cast<double>(msg->chassis_power_buffer) - this->buffer_min_level) /
                             (this->buffer_control_level - this->buffer_min_level);
    }
    this->scale_factor = std::min<double>(this->scale_factor, 1.0f);

}


void ChassisPowerControl::robot_status_callback(gary_msgs::msg::RobotStatus::SharedPtr msg) {
    this->max_power = msg->chassis_power_limit;
}


int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<ChassisPowerControl> chassis_power_control = std::make_shared<ChassisPowerControl>(
            rclcpp::NodeOptions());

    exe.add_node(chassis_power_control->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_chassis::ChassisPowerControl)


#include "gary_chsssis/odometry.hpp"

using namespace gary_chassis;

using namespace std::chrono_literals;

OdometryR::OdometryR(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("odom_node",
                                                                                                   options),a(0),b(0),r(0) {
    this->declare_parameter("odom_topic","/odom");
    this->declare_parameter("twist_topic");
    this->declare_parameter("joint_topic","/dynamic_joint_states");
    this->declare_parameter("imu_topic","//gimbal_imu_broadcaster/imu");
    this->declare_parameter("a");
    this->declare_parameter("b");
    this->declare_parameter("r");
}

CallbackReturn OdometryR::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //get joint
    if (this->get_parameter("joint_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "joint_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->joint_topic = this->get_parameter("joint_topic").as_string();
    this->joint_subscriber = this->create_subscription<control_msgs::msg::DynamicJointState>(
            this->joint_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&OdometryR::joint_callback, this, std::placeholders::_1));
    //get imu
    if (this->get_parameter("imu_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "imu_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->imu_topic = this->get_parameter("imu_topic").as_string();
    this->imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
            this->imu_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&OdometryR::imu_callback, this, std::placeholders::_1));

    //get the twist_msg
    if (this->get_parameter("twist_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "twist_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->twist_topic = this->get_parameter("twist_topic").as_string();
    this->twist_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            this->twist_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&OdometryR::twist_callback, this, std::placeholders::_1));


    //get odom publisher
    if (this->get_parameter("odom_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "odom_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->odom_topic = this->get_parameter("odom_topic").as_string();
    this->odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(this->odom_topic,
                                                                        rclcpp::SystemDefaultsQoS());
    //get time
    this->timer = this->create_wall_timer(100ms,std::bind(&OdometryR::time_callback,this));

    //get a
    if (this->get_parameter("a").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "a type must be double");
        return CallbackReturn::FAILURE;
    }
    this->a = this->get_parameter("a").as_double();

    //get b
    if (this->get_parameter("b").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "b type must be double");
        return CallbackReturn::FAILURE;
    }
    this->b = this->get_parameter("b").as_double();

    //get r
    if (this->get_parameter("r").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "r type must be double");
        return CallbackReturn::FAILURE;
    }
    this->r = this->get_parameter("r").as_double();

    this->mecanum_kinematics = std::make_shared<gary_chassis::MecanumKinematics>(this->a, this->b, this->r);
}

CallbackReturn OdometryR::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    this->odom_publisher.reset();
    this->twist_subscriber.reset();
    this->mecanum_kinematics.reset();
    RCLCPP_INFO(this->get_logger(), "cleaning up");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OdometryR::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    this->odom_publisher->on_activate();
    RCLCPP_INFO(this->get_logger(), "activated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn OdometryR::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    this->odom_publisher->on_deactivate();
    RCLCPP_INFO(this->get_logger(), "deactivated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn OdometryR::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    if (this->odom_publisher.get() != nullptr) this->odom_publisher.reset();
    if (this->twist_subscriber.get() != nullptr) this->twist_subscriber.reset();
    if (this->mecanum_kinematics != nullptr) this->mecanum_kinematics.reset();
    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OdometryR::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    if (this->odom_publisher.get() != nullptr) this->odom_publisher.reset();
    if (this->twist_subscriber.get() != nullptr) this->twist_subscriber.reset();
    if (this->mecanum_kinematics != nullptr) this->mecanum_kinematics.reset();
    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

void OdometryR::twist_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
    twist_msg = *msg;
}
void OdometryR::imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
{
    this->Imu = *msg;
    (tf2::Quaternion(Imu.orientation.x, Imu.orientation.y, Imu.orientation.z, Imu.orientation.w));
    tf2::Matrix3x3 (imu_quaternion);
}
void OdometryR::joint_callback(control_msgs::msg::DynamicJointState::SharedPtr joint_state) {
    for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
        if(joint_state->joint_names[i] == "chassis_lf")
        {
            for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j)
            {
                if (joint_state->interface_values[i].interface_names[j] == "ecd")
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
                if (joint_state->interface_values[i].interface_names[j] == "ecd")
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
                if (joint_state->interface_values[i].interface_names[j] == "ecd")
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
                if (joint_state->interface_values[i].interface_names[j] == "ecd")
                {
                    this->rb_speed = joint_state->interface_values[i].values[j];
                }
            }
        }
    }

}
void OdometryR::time_callback() {
    std::map<std::string, double> chassis_speed;
    std::map<std::string, double> wheel_speed;

    wheel_speed.emplace("left_back",lb_speed);
    wheel_speed.emplace("right_back",rb_speed);
    wheel_speed.emplace("left_front",lf_speed);
    wheel_speed.emplace("right_front",rf_speed);

    chassis_speed = mecanum_kinematics->forward_solve(wheel_speed,WHEEL_OFFLINE_NONE);

    nav_msgs::msg::Odometry data;
    data.twist.twist.linear.x = chassis_speed["vx"];
    data.twist.twist.linear.y = chassis_speed["vy"];
    data.pose.pose.position.x =chassis_speed["vx"];
    data.pose.pose.position.y =chassis_speed["vy"];

    this->odom_publisher->publish(data);
}
int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<OdometryR> odom_node = std::make_shared<OdometryR>(rclcpp::NodeOptions());

    exe.add_node(odom_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_chassis::OdometryR)
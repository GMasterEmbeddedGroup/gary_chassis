#include "gary_chsssis/chassis_autonomous.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


using namespace std::chrono_literals;

using namespace gary_chassis;


ChassisAutonomous::ChassisAutonomous(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("chassis_autonomous",
                                                                                                   options) {
    //declare params
    this->declare_parameter("twist_pub_topic", "/cmd_vel");
    this->declare_parameter("remote_control_topic", "/remote_control");
    this->declare_parameter("x_max_speed", 2.0f);
    this->declare_parameter("y_max_speed", 2.0f);
    this->declare_parameter("rotate_max_speed", 1.0f);
    this->declare_parameter("x_max_accel", 1.0f);
    this->declare_parameter("y_max_accel", 1.0f);
    this->declare_parameter("update_rate", 100.0f);
}

CallbackReturn ChassisAutonomous::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //create callback group
    this->cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;

    //get twist_pub_topic
    this->twist_pub_topic = this->get_parameter("twist_pub_topic").as_string();
    this->twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>(this->twist_pub_topic,
                                                                              rclcpp::SystemDefaultsQoS());

    //get remote_control_topic
    this->remote_control_topic = this->get_parameter("remote_control_topic").as_string();
    this->rc_subscriber = this->create_subscription<gary_msgs::msg::DR16Receiver>(
            this->remote_control_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisAutonomous::rc_callback, this, std::placeholders::_1), sub_options);
    this->rc_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

    //get x_max_speed
    this->x_max_speed = this->get_parameter("x_max_speed").as_double();

    //get y_max_speed
    this->y_max_speed = this->get_parameter("y_max_speed").as_double();

    //get rotate_max_speed
    this->rotate_max_speed = this->get_parameter("rotate_max_speed").as_double();

    //get x_max_accel
    this->x_max_accel = this->get_parameter("x_max_accel").as_double();
    this->x_filter = std::make_shared<gary_chassis::First_orderFilter>(this->x_max_accel);

    //get y_max_accel
    this->y_max_accel = this->get_parameter("y_max_accel").as_double();
    this->y_filter = std::make_shared<gary_chassis::First_orderFilter>(this->y_max_accel);

    this->a_filter = std::make_shared<gary_chassis::First_orderFilter>(8.0f);

    //get update_rate
    this->update_rate = this->get_parameter("update_rate").as_double();

    //odom
    this->odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry_2d", rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisAutonomous::odom_callback, this, std::placeholders::_1), sub_options);
    this->odom_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

    RCLCPP_INFO(this->get_logger(), "configured");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisAutonomous::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    this->twist_publisher.reset();
    this->rc_subscriber.reset();
    this->odom_subscriber.reset();
    this->x_filter.reset();
    this->y_filter.reset();
    this->a_filter.reset();

    RCLCPP_INFO(this->get_logger(), "cleaning up");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisAutonomous::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //activate lifecycle publisher
    this->twist_publisher->on_activate();

    //create timer
    this->timer_update = this->create_wall_timer(1000ms / this->update_rate, [this] { update(); }, this->cb_group);
    this->timer_decision = this->create_wall_timer(1000ms / this->update_rate, [this] { decision(); }, this->cb_group);

    this->decision_timestamp = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisAutonomous::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //deactivate lifecycle publisher
    this->twist_publisher->on_deactivate();

    //destroy timer
    this->timer_update.reset();
    this->timer_decision.reset();

    RCLCPP_INFO(this->get_logger(), "deactivated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisAutonomous::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->twist_publisher.get() != nullptr) this->twist_publisher.reset();
    if (this->rc_subscriber.get() != nullptr) this->rc_subscriber.reset();
    if (this->odom_subscriber.get() != nullptr) this->odom_subscriber.reset();
    if (this->x_filter != nullptr) this->x_filter.reset();
    if (this->y_filter != nullptr) this->y_filter.reset();
    if (this->a_filter != nullptr) this->a_filter.reset();
    if (this->timer_update != nullptr) this->timer_update->reset();
    if (this->timer_decision != nullptr) this->timer_decision->reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisAutonomous::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->twist_publisher.get() != nullptr) this->twist_publisher.reset();
    if (this->rc_subscriber.get() != nullptr) this->rc_subscriber.reset();
    if (this->odom_subscriber.get() != nullptr) this->odom_subscriber.reset();
    if (this->x_filter != nullptr) this->x_filter.reset();
    if (this->y_filter != nullptr) this->y_filter.reset();
    if (this->a_filter != nullptr) this->a_filter.reset();
    if (this->timer_update != nullptr) this->timer_update->reset();
    if (this->timer_decision != nullptr) this->timer_decision->reset();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}


void ChassisAutonomous::rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg) {
    this->rc = *msg;
    this->rc_timestamp = this->get_clock()->now();
}


void ChassisAutonomous::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    this->odom = *msg;
    this->odom_timestamp = this->get_clock()->now();
}

void ChassisAutonomous::update() {

    rclcpp::Time time_now = this->get_clock()->now();

    bool rc_available = (time_now - this->rc_timestamp).seconds() <= 0.5;
    bool odom_available = (time_now - this->odom_timestamp).seconds() <= 0.5;


    if (rc_available && odom_available) {
        geometry_msgs::msg::Twist twist;

        double rc_vx_set, rc_vy_set, rc_az_set;
        //joystick control
        rc_vx_set = this->rc.ch_left_y * x_max_speed;
        rc_vy_set = -this->rc.ch_left_x * y_max_speed;
        rc_az_set = -this->rc.ch_wheel * rotate_max_speed;

        double vx_set, vy_set, az_set;
        double x_err = x_set - this->odom.pose.pose.position.x;
        if (x_err > this->x_max_speed) x_err = this->x_max_speed;
        if (x_err < - this->x_max_speed) x_err = - this->x_max_speed;

        double y_err = y_set - this->odom.pose.pose.position.y;
        if (y_err > this->y_max_speed) y_err = this->y_max_speed;
        if (y_err < - this->y_max_speed) y_err = - this->y_max_speed;

        this->on_position = fabs(x_err) < 0.05 && fabs(y_err) < 0.05;

        vx_set = y_err;
        vy_set = -x_err;

        //get yaw angle
        if(this->odom.pose.pose.orientation.x == 0.0f && this->odom.pose.pose.orientation.y == 0.0f && this->odom.pose.pose.orientation.z == 0.0f && this->odom.pose.pose.orientation.w == 0.0f) return;
        double yaw, pitch, roll;
        //quat to euler
        tf2::Quaternion imu_quaternion(this->odom.pose.pose.orientation.x, this->odom.pose.pose.orientation.y, this->odom.pose.pose.orientation.z, this->odom.pose.pose.orientation.w);
        tf2::Matrix3x3 m(imu_quaternion);
        m.getRPY(roll, pitch, yaw);

        //speed filter
        vx_set = this->x_filter->first_order_filter(vx_set);
        vy_set = this->y_filter->first_order_filter(vy_set);
        az_set = this->a_filter->first_order_filter(this->z_set);

        //map vector to global direction
        double sin_yaw, cos_yaw;
        double gimbal_vx, gimbal_vy;
        sin_yaw = sin(yaw);
        cos_yaw = cos(yaw);
        gimbal_vx = cos_yaw * (vx_set + rc_vx_set) + sin_yaw * (vy_set + rc_vy_set);
        gimbal_vy = -sin_yaw * (vx_set + rc_vx_set) + cos_yaw * (vy_set + rc_vy_set);

        if (this->rc.sw_right == gary_msgs::msg::DR16Receiver::SW_DOWN) {
            return;
        } else if(this->rc.sw_right == gary_msgs::msg::DR16Receiver::SW_MID) {
            twist.linear.x = rc_vx_set;
            twist.linear.y = rc_vy_set;
            twist.angular.z = rc_az_set;
        } else if (this->rc.sw_right == gary_msgs::msg::DR16Receiver::SW_UP) {
            if(az_set == 0) {
                twist.linear.x = gimbal_vx;
                twist.linear.y = gimbal_vy;
                twist.angular.z = 0.0f;
            } else {
                twist.linear.x = 0.0f;
                twist.linear.y = 0.0f;
                twist.angular.z = az_set;
            }
        }

        this->twist_publisher->publish(twist);

        rclcpp::Clock clock;
//        RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 1000, "x %f y %f yaw %f onpos %d", this->odom.pose.pose.position.x, this->odom.pose.pose.position.y, yaw, this->on_position);
    }
}

void ChassisAutonomous::decision() {
    rclcpp::Time time_now = this->get_clock()->now();

    bool rc_available = (time_now - this->rc_timestamp).seconds() <= 0.5;
    bool odom_available = (time_now - this->odom_timestamp).seconds() <= 0.5;

    static int stage = 0;
    static float last_rc = 0.0f;
    static double x_set_temp, y_set_temp;

    if (rc_available && odom_available) {
        if (this->rc.sw_right == gary_msgs::msg::DR16Receiver::SW_DOWN) {
            return;
        } else if(this->rc.sw_right == gary_msgs::msg::DR16Receiver::SW_UP) {

            if (this->rc.ch_wheel != 0 && last_rc == 0.0f) {
                stage++;
                switch (stage) {
                    case 0: {
                        x_set_temp = 0.0f;
                        y_set_temp = 0.0f;
                        break;
                    }
                    case 1: {
                        x_set_temp = 2.7f;
                        y_set_temp = 2.7f;
                        break;
                    }
                    case 2: {
                        x_set_temp = 2.7f;
                        y_set_temp = 6.0f;
                        break;
                    }
                    case 3: {
                        x_set_temp = 2.7f;
                        y_set_temp = 2.7f;
                        break;
                    }
                    case 4: {
                        x_set_temp = 0.0f;
                        y_set_temp = 0.0f;
                        break;
                    }
                    case 5: {
                        x_set_temp = -4.2f;
                        y_set_temp = -2.4f;
                        break;
                    }
                    default: {
                        x_set_temp = 0.0f;
                        y_set_temp = 0.0f;
                        stage = 0;
                    }
                }
                this->decision_timestamp = this->get_clock()->now();
                RCLCPP_INFO(this->get_logger(), "new goal: x %f y %f", this->x_set, this->y_set);
            }
            last_rc = this->rc.ch_wheel;

            if (this->x_set != x_set_temp || this->y_set != y_set_temp) {
                this->z_set = 0.0f;
                if ((this->get_clock()->now() - this->decision_timestamp).seconds() > 2.0f) {
                    this->x_set = x_set_temp;
                    this->y_set = y_set_temp;
                }
            } else {
                if (this->on_position) {
                    this->z_set = this->rotate_max_speed;
                }
            }

        }
    }
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr <ChassisAutonomous> chassis_autonomous = std::make_shared<ChassisAutonomous>(rclcpp::NodeOptions());

    exe.add_node(chassis_autonomous->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_chassis::ChassisAutonomous)

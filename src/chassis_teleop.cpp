#include "gary_chsssis/chassis_teleop.hpp"

using namespace std::chrono_literals;

using namespace gary_chassis;


ChassisTeleop::ChassisTeleop(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("chassis_teleop",
                                                                                                   options) {
    //declare params
    this->declare_parameter("twist_pub_topic", "/cmd_vel");
    this->declare_parameter("remote_control_topic", "/remote_control");
    this->declare_parameter("diagnostic_topic", "/diagnostics_agg");
    this->declare_parameter("joint_topic", "/dynamic_joint_states");
    this->declare_parameter("gimbal_follow_set_topic", "/gimbal_follow_pid/cmd");
    this->declare_parameter("gimbal_follow_fdb_topic", "/gimbal_follow_pid/pid");
    this->declare_parameter("motor_yaw_hw_id","");
    this->declare_parameter("x_max_speed", 2.0f);
    this->declare_parameter("y_max_speed", 2.0f);
    this->declare_parameter("rotate_max_speed", 1.0f);
    this->declare_parameter("x_max_accel", 1.0f);
    this->declare_parameter("y_max_accel", 1.0f);
    this->declare_parameter("use_break", true);
    this->declare_parameter("update_rate", 200.0f);
    this->declare_parameter("yaw_encoder_bias", 0.0f);
}

CallbackReturn ChassisTeleop::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //create callback group
    this->cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;

    //get cmd_topic
    this->twist_pub_topic = this->get_parameter("twist_pub_topic").as_string();
    this->twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>(this->twist_pub_topic,
                                                                              rclcpp::SystemDefaultsQoS());
    //get diag
    this->diagnostic_topic = this->get_parameter("diagnostic_topic").as_string();
    this->diag_subscriber = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            this->diagnostic_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisTeleop::diag_callback, this, std::placeholders::_1), sub_options);

    //get remote_control_topic
    this->remote_control_topic = this->get_parameter("remote_control_topic").as_string();
    this->rc_subscriber = this->create_subscription<gary_msgs::msg::DR16Receiver>(
            this->remote_control_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisTeleop::rc_callback, this, std::placeholders::_1), sub_options);
    this->rc_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

    //get joint_topic
    this->joint_topic = this->get_parameter("joint_topic").as_string();
    this->joint_subscriber = this->create_subscription<control_msgs::msg::DynamicJointState>(
            this->joint_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisTeleop::joint_callback, this, std::placeholders::_1), sub_options);
    this->joint_state_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

    //get gimbal_follow_set_topic
    this->gimbal_follow_set_topic = this->get_parameter("gimbal_follow_set_topic").as_string();
    this->gimbal_follow_set_publisher = this->create_publisher<std_msgs::msg::Float64>(this->gimbal_follow_set_topic,
                                                                                       rclcpp::SystemDefaultsQoS());

    //get gimbal_follow_fdb_topic
    this->gimbal_follow_fdb_topic = this->get_parameter("gimbal_follow_fdb_topic").as_string();
    this->gimbal_follow_sub = this->create_subscription<gary_msgs::msg::PID>(
            this->gimbal_follow_fdb_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisTeleop::gimbal_follow_callback, this, std::placeholders::_1), sub_options);
    this->gimbal_follow_pid_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

    //get offline yaw if
    this->motor_yaw_hw_id = this->get_parameter("motor_yaw_hw_id").as_string();

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

    //get use_break
    this->use_break = this->get_parameter("use_break").as_bool();

    //get update_rate
    this->update_rate = this->get_parameter("update_rate").as_double();

    //get yaw_encoder_bias
    this->yaw_encoder_bias = this->get_parameter("yaw_encoder_bias").as_double();

    RCLCPP_INFO(this->get_logger(), "configured");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    this->twist_publisher.reset();
    this->gimbal_follow_set_publisher.reset();
    this->diag_subscriber.reset();
    this->rc_subscriber.reset();
    this->joint_subscriber.reset();
    this->gimbal_follow_sub.reset();
    this->x_filter.reset();
    this->y_filter.reset();

    RCLCPP_INFO(this->get_logger(), "cleaning up");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //activate lifecycle publisher
    this->twist_publisher->on_activate();
    this->gimbal_follow_set_publisher->on_activate();

    //set chassis mode initial state
    this->chassis_mode = CHASSIS_MODE_ZERO_FORCE;
    this->last_chassis_mode = CHASSIS_MODE_NORMAL;
    this->last_sw_state = gary_msgs::msg::DR16Receiver::SW_DOWN;

    //create timer
    this->timer_update = this->create_wall_timer(1000ms / this->update_rate, [this] { update(); }, this->cb_group);

    RCLCPP_INFO(this->get_logger(), "activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //deactivate lifecycle publisher
    this->twist_publisher->on_deactivate();
    this->gimbal_follow_set_publisher->on_deactivate();

    //destroy timer
    this->timer_update.reset();

    RCLCPP_INFO(this->get_logger(), "deactivated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->twist_publisher.get() != nullptr) this->twist_publisher.reset();
    if (this->diag_subscriber.get() != nullptr) this->diag_subscriber.reset();
    if (this->gimbal_follow_set_publisher.get() != nullptr) this->gimbal_follow_set_publisher.reset();
    if (this->rc_subscriber.get() != nullptr) this->rc_subscriber.reset();
    if (this->joint_subscriber.get() != nullptr) this->joint_subscriber.reset();
    if (this->gimbal_follow_sub.get() != nullptr) this->gimbal_follow_sub.reset();
    if (this->x_filter != nullptr) this->x_filter.reset();
    if (this->y_filter != nullptr) this->y_filter.reset();
    if (this->timer_update != nullptr) this->timer_update->reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->twist_publisher.get() != nullptr) this->twist_publisher.reset();
    if (this->diag_subscriber.get() != nullptr) this->diag_subscriber.reset();
    if (this->gimbal_follow_set_publisher.get() != nullptr) this->gimbal_follow_set_publisher.reset();
    if (this->rc_subscriber.get() != nullptr) this->rc_subscriber.reset();
    if (this->joint_subscriber.get() != nullptr) this->joint_subscriber.reset();
    if (this->gimbal_follow_sub.get() != nullptr) this->gimbal_follow_sub.reset();
    if (this->x_filter != nullptr) this->x_filter.reset();
    if (this->y_filter != nullptr) this->y_filter.reset();
    if (this->timer_update != nullptr) this->timer_update->reset();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}

void ChassisTeleop::gimbal_follow_callback(gary_msgs::msg::PID::SharedPtr msg) {
    this->gimbal_follow_pid = *msg;
    this->gimbal_follow_pid_timestamp = this->get_clock()->now();
}

void ChassisTeleop::joint_callback(control_msgs::msg::DynamicJointState::SharedPtr msg) {
    this->joint_state = *msg;
    this->joint_state_timestamp = this->get_clock()->now();
}

void ChassisTeleop::rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg) {
    this->rc = *msg;
    this->rc_timestamp = this->get_clock()->now();
}

void ChassisTeleop::diag_callback(diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    //store the data
    this->diagnostic_array = *msg;
}

void ChassisTeleop::update() {

    rclcpp::Time time_now = this->get_clock()->now();

    bool gimbal_follow_pid_available = (time_now - this->gimbal_follow_pid_timestamp).seconds() <= 0.5;
    bool joint_state_available = (time_now - this->joint_state_timestamp).seconds() <= 0.5;
    bool rc_available = (time_now - this->rc_timestamp).seconds() <= 0.5;

    double relative_angle;
    bool yaw_flag = false;
    //get offline yaw if
    for(const auto&i : this->diagnostic_array.status) {
        if (i.hardware_id == this->motor_yaw_hw_id && i.level == diagnostic_msgs::msg::DiagnosticStatus::OK) yaw_flag = true;
    }
    if (joint_state_available) {

        bool offline = true;
        //get yaw motor encoder and calc relative_angle
        for (unsigned long i = 0; i < this->joint_state.joint_names.size(); ++i) {
            if (this->joint_state.joint_names[i] == "gimbal_yaw") {
                for (unsigned long j = 0; j < this->joint_state.interface_values[i].interface_names.size(); ++j) {

                    if (this->joint_state.interface_values[i].interface_names[j] == "offline" && this->joint_state.interface_values[i].values[j] == 0.0f) {
                        offline = false;
                    }

                    if (this->joint_state.interface_values[i].interface_names[j] == "encoder") {
                        relative_angle = this->yaw_encoder_bias - this->joint_state.interface_values[i].values[j];

                    }
                }
            }
        }
        static bool flag_first = false;
        static double encoder_position_transform = 0;

        //calc encoder_position_transform in the first time
        if (!flag_first && !offline) {
            encoder_position_transform = relative_angle;
            flag_first = true;
        }

        //pub gimbal_follow_pid set
        std_msgs::msg::Float64 angle_data;
        double pid_set = encoder_position_transform;

        //prevent chassis spin multiple times when following gimbal
        if (gimbal_follow_pid_available) {
            while (pid_set - this->gimbal_follow_pid.feedback > M_PI) pid_set -= 2 * M_PI;
            while (pid_set - this->gimbal_follow_pid.feedback < -M_PI) pid_set += 2 * M_PI;
        }

        angle_data.data = pid_set;
        this->gimbal_follow_set_publisher->publish(angle_data);
    }

    if (rc_available) {
        double vx_set, vy_set, az_set;
        geometry_msgs::msg::Twist twist;

        //mode switch
        if (this->rc.sw_right == gary_msgs::msg::DR16Receiver::SW_DOWN &&
            this->last_sw_state == gary_msgs::msg::DR16Receiver::SW_MID) {

            //switch from middle to down
            this->last_chassis_mode = this->chassis_mode;
            this->chassis_mode = CHASSIS_MODE_ZERO_FORCE;
            RCLCPP_INFO(this->get_logger(), "enter zero force mode");

        } else if (this->rc.sw_right == gary_msgs::msg::DR16Receiver::SW_MID &&
                   this->last_sw_state == gary_msgs::msg::DR16Receiver::SW_DOWN) {

            //switch from down to middle
            this->chassis_mode = this->last_chassis_mode;
            RCLCPP_INFO(this->get_logger(), "exit zero force mode");

        } else if (this->rc.sw_right == gary_msgs::msg::DR16Receiver::SW_MID &&
                   this->last_sw_state == gary_msgs::msg::DR16Receiver::SW_UP) {

            //switch from up to middle
            switch (this->chassis_mode) {
                case CHASSIS_MODE_ZERO_FORCE:
                    break;
                case CHASSIS_MODE_NORMAL:
                    if(yaw_flag) {
                        this->chassis_mode = CHASSIS_MODE_FOLLOW_GIMBAL;
                        RCLCPP_INFO(this->get_logger(), "switch to follow gimbal mode");
                    }else
                    {
                        this->chassis_mode = CHASSIS_MODE_NORMAL;
                        RCLCPP_INFO(this->get_logger(), "switch failed");
                    }
                    break;
                case CHASSIS_MODE_FOLLOW_GIMBAL:
                    this->chassis_mode = CHASSIS_MODE_SPIN;
                    RCLCPP_INFO(this->get_logger(), "switch to spin mode");
                    break;
                case CHASSIS_MODE_SPIN:
                    this->chassis_mode = CHASSIS_MODE_NORMAL;
                    RCLCPP_INFO(this->get_logger(), "switch to normal mode");
                    break;
            }
        }
        this->last_sw_state = this->rc.sw_right;

        if (!joint_state_available && !gimbal_follow_pid_available &&
            (this->chassis_mode == CHASSIS_MODE_FOLLOW_GIMBAL || this->chassis_mode == CHASSIS_MODE_SPIN)) {
            this->chassis_mode = CHASSIS_MODE_NORMAL;
            RCLCPP_WARN(this->get_logger(), "switch to normal mode due to data unavailable");
        }

        //joystick control
        vx_set = this->rc.ch_left_y * x_max_speed;
        vy_set = -this->rc.ch_left_x * y_max_speed;
        az_set = -this->rc.ch_wheel * rotate_max_speed;

        //keyboard control
        bool swing_flag = false;
        if (this->rc.key_w) {
            vx_set = this->y_max_speed;
        } else if (this->rc.key_s) {
            vx_set = -this->y_max_speed;
        } else if (this->rc.key_a) {
            vy_set = this->y_max_speed;
        } else if (this->rc.key_d) {
            vy_set = -this->y_max_speed;
        }
        if (this->rc.key_shift) {
            az_set = this->rotate_max_speed;
            swing_flag = true;
        }

        //speed filter
        if (this->use_break && vx_set == 0) this->x_filter->reset();
        if (this->use_break && vy_set == 0) this->y_filter->reset();
        vx_set = this->x_filter->first_order_filter(vx_set);
        vy_set = this->y_filter->first_order_filter(vy_set);

        //map vector to gimbal direction
        double sin_yaw, cos_yaw;
        double gimbal_vx, gimbal_vy;
        sin_yaw = sin(relative_angle);
        cos_yaw = cos(relative_angle);
        gimbal_vx = cos_yaw * vx_set + sin_yaw * vy_set;
        gimbal_vy = -sin_yaw * vx_set + cos_yaw * vy_set;

        switch (this->chassis_mode) {
            case CHASSIS_MODE_ZERO_FORCE:
                return;
            case CHASSIS_MODE_NORMAL:
                twist.linear.x = vx_set;
                twist.linear.y = vy_set;
                twist.angular.z = az_set;
                break;
            case CHASSIS_MODE_FOLLOW_GIMBAL:
                twist.linear.x = gimbal_vx;
                twist.linear.y = gimbal_vy;
                if(!swing_flag)
                {
                    twist.angular.z = this->gimbal_follow_pid.out;
                }else
                {
                    twist.angular.z = rotate_max_speed;
                }

                break;
            case CHASSIS_MODE_SPIN:
                twist.linear.x = gimbal_vx;
                twist.linear.y = gimbal_vy;
                twist.angular.z = rotate_max_speed;
                break;
        }

        this->twist_publisher->publish(twist);
    }
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr <ChassisTeleop> chassis_teleop = std::make_shared<ChassisTeleop>(rclcpp::NodeOptions());

    exe.add_node(chassis_teleop->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_chassis::ChassisTeleop)

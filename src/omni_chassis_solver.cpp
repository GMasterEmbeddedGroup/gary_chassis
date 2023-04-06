#include "gary_chsssis/omni_chassis_solver.hpp"


using namespace gary_chassis;

OmniChassisSolver::OmniChassisSolver(const rclcpp::NodeOptions &options) :
        rclcpp_lifecycle::LifecycleNode("omni_chassis_solver", options), a(0), b(0), r(0) {
    //declare param
    this->declare_parameter("cmd_topic", "~/cmd_vel");
    this->declare_parameter("diagnostic_topic", "/diagnostics_agg");
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("joint_topic", "/dynamic_joint_states");
    this->declare_parameter("output_lf_topic");
    this->declare_parameter("output_lb_topic");
    this->declare_parameter("output_rf_topic");
    this->declare_parameter("output_rb_topic");
    this->declare_parameter("motor_lf_hw_id");
    this->declare_parameter("motor_lb_hw_id");
    this->declare_parameter("motor_rf_hw_id");
    this->declare_parameter("motor_rb_hw_id");
    this->declare_parameter("a");
    this->declare_parameter("b");
    this->declare_parameter("r");
}

CallbackReturn OmniChassisSolver::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //get cmd_topic
    if (this->get_parameter("cmd_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "cmd_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->cmd_topic = this->get_parameter("cmd_topic").as_string();
    this->cmd_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            this->cmd_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&OmniChassisSolver::cmd_callback, this, std::placeholders::_1));

    //get joint_topic
    if (this->get_parameter("joint_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "joint_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->joint_topic = this->get_parameter("joint_topic").as_string();
    this->joint_sub = this->create_subscription<control_msgs::msg::DynamicJointState>(
            this->joint_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&OmniChassisSolver::joint_callback, this, std::placeholders::_1));


    //get diagnostic_topic
    if (this->get_parameter("diagnostic_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "diagnostic_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->diagnostic_topic = this->get_parameter("diagnostic_topic").as_string();
    this->diag_subscriber = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            this->diagnostic_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&OmniChassisSolver::diag_callback, this, std::placeholders::_1));

    //get odom_topic
    if (this->get_parameter("odom_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "odom_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->omni_odom_topic = this->get_parameter("odom_topic").as_string();
    this->omni_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(this->omni_odom_topic,
                                                                           rclcpp::SystemDefaultsQoS());


    //get output_lf_topic
    if (this->get_parameter("output_lf_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "output_lf_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->output_lf_topic = this->get_parameter("output_lf_topic").as_string();
    this->lf_publisher = this->create_publisher<std_msgs::msg::Float64>(this->output_lf_topic,
                                                                        rclcpp::SystemDefaultsQoS());

    //get output_lb_topic
    if (this->get_parameter("output_lb_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "output_lb_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->output_lb_topic = this->get_parameter("output_lb_topic").as_string();
    this->lb_publisher = this->create_publisher<std_msgs::msg::Float64>(this->output_lb_topic,
                                                                        rclcpp::SystemDefaultsQoS());

    //get output_rf_topic
    if (this->get_parameter("output_rf_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "output_rf_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->output_rf_topic = this->get_parameter("output_rf_topic").as_string();
    this->rf_publisher = this->create_publisher<std_msgs::msg::Float64>(this->output_rf_topic,
                                                                        rclcpp::SystemDefaultsQoS());

    //get output_rb_topic
    if (this->get_parameter("output_rb_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "output_rb_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->output_rb_topic = this->get_parameter("output_rb_topic").as_string();
    this->rb_publisher = this->create_publisher<std_msgs::msg::Float64>(this->output_rb_topic,
                                                                        rclcpp::SystemDefaultsQoS());

    //get motor_lf_hw_id
    if (this->get_parameter("motor_lf_hw_id").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "motor_lf_hw_id type must be string");
        return CallbackReturn::FAILURE;
    }
    this->motor_lf_hw_id = this->get_parameter("motor_lf_hw_id").as_string();

    //get motor_lb_hw_id
    if (this->get_parameter("motor_lb_hw_id").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "motor_lb_hw_id type must be string");
        return CallbackReturn::FAILURE;
    }
    this->motor_lb_hw_id = this->get_parameter("motor_lb_hw_id").as_string();

    //get motor_rf_hw_id
    if (this->get_parameter("motor_rf_hw_id").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "motor_rf_hw_id type must be string");
        return CallbackReturn::FAILURE;
    }
    this->motor_rf_hw_id = this->get_parameter("motor_rf_hw_id").as_string();

    //get motor_rb_hw_id
    if (this->get_parameter("motor_rb_hw_id").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "motor_rb_hw_id type must be string");
        return CallbackReturn::FAILURE;
    }
    this->motor_rb_hw_id = this->get_parameter("motor_rb_hw_id").as_string();

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

    this->omni_kinematics = std::make_shared<gary_chassis::OmniKinematics>(this->a, this->b, this->r);

    RCLCPP_INFO(this->get_logger(), "configured");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniChassisSolver::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    this->cmd_subscriber.reset();
    this->omni_odom_publisher.reset();
    this->diag_subscriber.reset();
    this->lf_publisher.reset();
    this->lb_publisher.reset();
    this->rf_publisher.reset();
    this->rb_publisher.reset();
    this->omni_kinematics.reset();

    RCLCPP_INFO(this->get_logger(), "cleaning up");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniChassisSolver::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    this->omni_odom_publisher->on_activate();
    this->lf_publisher->on_activate();
    this->lb_publisher->on_activate();
    this->rf_publisher->on_activate();
    this->rb_publisher->on_activate();
    this->omni_last_time = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniChassisSolver::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    this->omni_odom_publisher->on_deactivate();
    this->lf_publisher->on_deactivate();
    this->lb_publisher->on_deactivate();
    this->rf_publisher->on_deactivate();
    this->rb_publisher->on_deactivate();

    RCLCPP_INFO(this->get_logger(), "deactivated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniChassisSolver::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    if (this->omni_odom_publisher.get() != nullptr) this->omni_odom_publisher.reset();
    if (this->joint_sub.get() != nullptr) this->joint_sub.reset();
    if (this->cmd_subscriber.get() != nullptr) this->cmd_subscriber.reset();
    if (this->diag_subscriber.get() != nullptr) this->diag_subscriber.reset();
    if (this->lf_publisher.get() != nullptr) this->lf_publisher.reset();
    if (this->lb_publisher.get() != nullptr) this->lb_publisher.reset();
    if (this->rf_publisher.get() != nullptr) this->rf_publisher.reset();
    if (this->rb_publisher.get() != nullptr) this->rb_publisher.reset();
    if (this->omni_kinematics != nullptr) this->omni_kinematics.reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");

    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniChassisSolver::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    if (this->joint_sub.get() != nullptr) this->joint_sub.reset();
    if (this->omni_odom_publisher.get() != nullptr) this->omni_odom_publisher.reset();
    if (this->cmd_subscriber.get() != nullptr) this->cmd_subscriber.reset();
    if (this->diag_subscriber.get() != nullptr) this->diag_subscriber.reset();
    if (this->lf_publisher.get() != nullptr) this->lf_publisher.reset();
    if (this->lb_publisher.get() != nullptr) this->lb_publisher.reset();
    if (this->rf_publisher.get() != nullptr) this->rf_publisher.reset();
    if (this->rb_publisher.get() != nullptr) this->rb_publisher.reset();
    if (this->omni_kinematics != nullptr) this->omni_kinematics.reset();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}
void OmniChassisSolver::joint_callback(control_msgs::msg::DynamicJointState::SharedPtr joint_state) {
    for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
        if(joint_state->joint_names[i] == "chassis_left_front")
        {
            for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j)
            {
                if (joint_state->interface_values[i].interface_names[j] == "velocity")
                {
                    this->lf_speed = joint_state->interface_values[i].values[j];
                }
            }
        }
    }
    for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
        if(joint_state->joint_names[i] == "chassis_left_back")
        {
            for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j)
            {
                if (joint_state->interface_values[i].interface_names[j] == "velocity")
                {
                    this->lb_speed = joint_state->interface_values[i].values[j];
                }
            }
        }
    }
    for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
        if(joint_state->joint_names[i] == "chassis_right_front")
        {
            for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j)
            {
                if (joint_state->interface_values[i].interface_names[j] == "velocity")
                {
                    this->rf_speed = joint_state->interface_values[i].values[j];
                }
            }
        }
    }
    for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
        if(joint_state->joint_names[i] == "chassis_right_back")
        {
            for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j)
            {
                if (joint_state->interface_values[i].interface_names[j] == "velocity")
                {
                    this->rb_speed = joint_state->interface_values[i].values[j];
                }
            }
        }

    }
    //RCLCPP_INFO(this->get_logger(), "lb %f lf %f rb%f rf%", this->lb_speed,this->lf_speed,rb_speed,rf_speed);
}
void OmniChassisSolver::cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
    std::map<std::string, double> chassis_speed;
    std::map<std::string, double> wheel_speed;
    std::map<std::string, double> chassis_speed_odom;
    std::map<std::string, double> wheel_speed_odom;

    chassis_speed.emplace("vx", msg->linear.x);
    chassis_speed.emplace("vy", msg->linear.y);
    chassis_speed.emplace("az", msg->angular.z);

    wheel_speed_odom.emplace("left_back",lb_speed);
    wheel_speed_odom.emplace("right_back",rb_speed);
    wheel_speed_odom.emplace("left_front",lf_speed);
    wheel_speed_odom.emplace("right_front",rf_speed);

    bool lf_online_flag, lb_online_flag, rf_online_flag, rb_online_flag;
    lf_online_flag = lb_online_flag = rf_online_flag = rb_online_flag = false;

    //get offline motor
    for(const auto&i : this->diagnostic_array.status) {
        if (i.hardware_id == this->motor_lf_hw_id && i.level == diagnostic_msgs::msg::DiagnosticStatus::OK) lf_online_flag = true;
        if (i.hardware_id == this->motor_lb_hw_id && i.level == diagnostic_msgs::msg::DiagnosticStatus::OK) lb_online_flag = true;
        if (i.hardware_id == this->motor_rf_hw_id && i.level == diagnostic_msgs::msg::DiagnosticStatus::OK) rf_online_flag = true;
        if (i.hardware_id == this->motor_rb_hw_id && i.level == diagnostic_msgs::msg::DiagnosticStatus::OK) rb_online_flag = true;
    }

    //recalculation when hardware offline
    if (lf_online_flag && lb_online_flag && rf_online_flag && rb_online_flag) {
        wheel_speed = this->omni_kinematics->inverse_solve(chassis_speed, WHEEL_OFFLINE_NONE);
    } else if (! lf_online_flag) {
        wheel_speed = this->omni_kinematics->inverse_solve(chassis_speed, WHEEL_OFFLINE_LF);
    } else if (! lb_online_flag) {
        wheel_speed = this->omni_kinematics->inverse_solve(chassis_speed, WHEEL_OFFLINE_LB);
    } else if (! rf_online_flag) {
        wheel_speed = this->omni_kinematics->inverse_solve(chassis_speed, WHEEL_OFFLINE_RF);
    } else if (! rb_online_flag) {
        wheel_speed = this->omni_kinematics->inverse_solve(chassis_speed, WHEEL_OFFLINE_RB);
    }
    //odometry calculate
    //chassis_speed_odom = omni_kinematics->forward_solve(wheel_speed_odom,WHEEL_OFFLINE_NONE);


    std_msgs::msg::Float64 data;
  /*  nav_msgs::msg::Odometry omni_odom_data;
    omni_current_time = this->get_clock()->now();
    double dt = 0;
    dt =  (omni_current_time - omni_last_time).seconds();
    vx_o = chassis_speed_odom["vx"];
    vy_o = chassis_speed_odom["vy"];
    az_o = chassis_speed_odom["az"];
    //RCLCPP_INFO(this->get_logger(), "az %f vx_o %f vy_o %f", az_o,vx_o,vy_o);
    double delta_x = (vx_o * cos(z_angle) - vy_o * sin(z_angle)) * dt;
    double delta_y = (vx_o * sin(z_angle) + vy_o * cos(z_angle)) * dt;
    double delta_z = az_o * dt;
    x += delta_x;
    y += delta_y;
    z += delta_z;
    if(z>0)z_angle = z+0.16;
    if(z<0)z_angle = z-0.17;

    //RCLCPP_INFO(this->get_logger(), "x %f y %f z %f z_angle %f", x, y, z, z_angle);

    omni_odom_data.header.frame_id = "odom";
    omni_odom_data.header.stamp = omni_current_time;
    omni_odom_data.pose.pose.position.x = x;
    omni_odom_data.pose.pose.position.y = y;
    omni_odom_data.pose.pose.position.z = 0.0;
    omni_odom_data.child_frame_id = "base_link";
    omni_odom_data.twist.twist.linear.x = chassis_speed_odom["vx"];
    omni_odom_data.twist.twist.linear.y = chassis_speed_odom["vy"];
    omni_odom_data.twist.twist.angular.z = chassis_speed_odom["az"];
*/
    //publish the needed motor msg
    if (wheel_speed.count("left_front") == 1) {
        data.data = wheel_speed["left_front"];
        this->lf_publisher->publish(data);
    }
    if (wheel_speed.count("left_back") == 1) {
        data.data = wheel_speed["left_back"];
        this->lb_publisher->publish(data);
    }
    if (wheel_speed.count("right_front") == 1) {
        data.data = wheel_speed["right_front"];
        this->rf_publisher->publish(data);
    }
    if (wheel_speed.count("right_back") == 1) {
        data.data = wheel_speed["right_back"];
        this->rb_publisher->publish(data);
    }
    //this->omni_odom_publisher->publish(omni_odom_data);
    //omni_last_time = omni_current_time;
}


void OmniChassisSolver::diag_callback(diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    //store the data
    this->diagnostic_array = *msg;
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<OmniChassisSolver> omni_chassis_solver = std::make_shared<OmniChassisSolver>(rclcpp::NodeOptions());

    exe.add_node(omni_chassis_solver->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_chassis::OmniChassisSolver)

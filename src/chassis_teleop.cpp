
#include "gary_chsssis/chassis_teleop.hpp"

using namespace std::chrono_literals;

using namespace gary_chassis;


ChassisTeleop::ChassisTeleop(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("chassis_teleop",
                                                                                                   options) {

    this->declare_parameter("cmd_topic");
    this->declare_parameter("remote_control", "/remote_control");
    this->declare_parameter("joint_topic","/dynamic_joint_states");
    this->declare_parameter("angle_follow","/relative_angle_pid/pid");
    this->declare_parameter("angle_set_topic","/relative_angle_pid/cmd");
    this->declare_parameter("x_max_speed");
    this->declare_parameter("y_max_speed");
    this->declare_parameter("rotate_max_speed");
    this->declare_parameter("x_max_accel");
    this->declare_parameter("y_max_accel");
}

CallbackReturn ChassisTeleop::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //get topic_cmd
    if (this->get_parameter("cmd_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "cmd_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->cmd_topic = this->get_parameter("cmd_topic").as_string();
    this->cmd_publisher = this->create_publisher<geometry_msgs::msg::Twist>(this->cmd_topic,
                                                                            rclcpp::SystemDefaultsQoS());
    //get angle_pid_set
    if (this->get_parameter("angle_set_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "angle_set_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->angle_set_topic = this->get_parameter("angle_set_topic").as_string();
    this->angle_pid_set_pub = this->create_publisher<std_msgs::msg::Float64>(this->angle_set_topic,
                                                                            rclcpp::SystemDefaultsQoS());
    //get remote_control
    if (this->get_parameter("remote_control").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "remote_control type must be string");
        return CallbackReturn::FAILURE;
    }
    this->remote_control_topic = this->get_parameter("remote_control").as_string();
    this->rc_subscriber = this->create_subscription<gary_msgs::msg::DR16Receiver>(
            this->remote_control_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisTeleop::rc_callback, this, std::placeholders::_1));

    //get angle_follow
    if (this->get_parameter("angle_follow").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "angle_follow type must be string");
        return CallbackReturn::FAILURE;
    }
    this->angle_follow = this->get_parameter("angle_follow").as_string();
    this->angle_follow_sub = this->create_subscription<gary_msgs::msg::PID>(
            this->angle_follow, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisTeleop::angle_follow_callback, this, std::placeholders::_1));

    //get joint
    if (this->get_parameter("joint_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "joint_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->joint_topic = this->get_parameter("joint_topic").as_string();
    this->joint_subscriber = this->create_subscription<control_msgs::msg::DynamicJointState>(
            this->joint_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisTeleop::joint_callback, this, std::placeholders::_1));

    //get y_max_speed
    if (this->get_parameter("y_max_speed").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "y_max_speed type must be double");
        return CallbackReturn::FAILURE;
    }
    this->y_max_speed = this->get_parameter("y_max_speed").as_double();

    //get x_max_speed
    if (this->get_parameter("x_max_speed").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "x_max_speed type must be double");
        return CallbackReturn::FAILURE;
    }
    this->x_max_speed = this->get_parameter("x_max_speed").as_double();

    //get rotate_max_speed
    if (this->get_parameter("rotate_max_speed").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "rotate_max_speed type must be double");
        return CallbackReturn::FAILURE;
    }
    this->rotate_max_speed = this->get_parameter("rotate_max_speed").as_double();

    //get x_max_accel
    if (this->get_parameter("x_max_accel").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "x_max_accel type must be double");
        return CallbackReturn::FAILURE;
    }
    this->x_max_accel = this->get_parameter("x_max_accel").as_double();

    //get y_max_accel
    if (this->get_parameter("y_max_accel").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "y_max_accel type must be double");
        return CallbackReturn::FAILURE;
    }
    this->y_max_accel = this->get_parameter("y_max_accel").as_double();

    this->x_filter = std::make_shared<gary_chassis::First_orderFilter>(this->x_max_accel);
    this->y_filter = std::make_shared<gary_chassis::First_orderFilter>(this->y_max_accel);

    RCLCPP_INFO(this->get_logger(), "configured");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    this->cmd_publisher.reset();
    this->angle_pid_set_pub.reset();
    this->rc_subscriber.reset();
    this->angle_follow_sub.reset();
    this->x_filter.reset();
    this->y_filter.reset();
    RCLCPP_INFO(this->get_logger(), "cleaning up");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    this->cmd_publisher->on_activate();
    this->angle_pid_set_pub->on_activate();
    RCLCPP_INFO(this->get_logger(), "activated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    this->cmd_publisher->on_deactivate();
    this->angle_pid_set_pub->on_deactivate();
    RCLCPP_INFO(this->get_logger(), "deactivated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);
    if (this->cmd_publisher.get() != nullptr) this->cmd_publisher.reset();
    if (this->angle_pid_set_pub.get() != nullptr) this->angle_pid_set_pub.reset();
    if (this->rc_subscriber.get() != nullptr) this->rc_subscriber.reset();
    if (this->angle_follow_sub.get() != nullptr) this->angle_follow_sub.reset();
    if (this->x_filter != nullptr) this->x_filter.reset();
    if (this->y_filter != nullptr) this->y_filter.reset();
    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisTeleop::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    if (this->cmd_publisher.get() != nullptr) this->cmd_publisher.reset();
    if (this->angle_pid_set_pub.get() != nullptr) this->angle_pid_set_pub.reset();
    if (this->rc_subscriber.get() != nullptr) this->rc_subscriber.reset();
    if (this->angle_follow_sub.get() != nullptr) this->angle_follow_sub.reset();
    if (this->x_filter != nullptr) this->x_filter.reset();
    if (this->y_filter != nullptr) this->y_filter.reset();
    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}

void ChassisTeleop::angle_follow_callback(gary_msgs::msg::PID::SharedPtr msg) {
    angle_follow_pid = *msg;
}
void ChassisTeleop::joint_callback(control_msgs::msg::DynamicJointState::SharedPtr joint_state) {
    for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
        if (joint_state->joint_names[i] == "gimbal_yaw") {
            for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j) {
                if (joint_state->interface_values[i].interface_names[j] == "encoder") {
                    encoder = joint_state->interface_values[i].values[j];
                    double origin = joint_state->interface_values[i].values[j];
                    double fixed = origin;
                    fixed = origin + 1.222581;
                    static int flag = 0;
                    if (flag == 0) {
                        gary_chassis::yaw.relative_angle_pre = origin;
                        flag = 1;
                    }

                    if (fixed < 0) fixed += 6.28;
                    if (fixed > PI) fixed -= 2 * PI;//转换为-PI到PI
                    if (origin > PI) origin -= 2 * PI;
                    gary_chassis::yaw.relative_angle = origin;
                    //RCLCPP_INFO(this->get_logger(), "origin %f fixed %f", gimbal::yaw.relative_angle, fixed);
                }
            }
        }
    }

    for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
        if (joint_state->joint_names[i] == "gimbal_yaw") {
            for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j) {
                if (joint_state->interface_values[i].interface_names[j] == "position") {
                    position = joint_state->interface_values[i].values[j];
                }
            }
        }
    }
    for (unsigned long i = 0; i < joint_state->joint_names.size(); ++i) {
        if (joint_state->joint_names[i] == "gimbal_yaw") {
            for (unsigned long j = 0; j < joint_state->interface_values[i].interface_names.size(); ++j) {
                if (joint_state->interface_values[i].interface_names[j] == "encoder_raw") {
                    ecd = joint_state->interface_values[i].values[j];
                }
            }
        }
    }

    if(!angle_pid_set_pub->is_activated()) return;
    std_msgs::msg::Float64 angle_data;
    static int f2 = 0;
    if(!f2)
    {
        foward_position = abs(encoder - 5.102787);
        f2 = 1;
    }

   if(encoder > 5.102787){
        angle_data.data = -foward_position;
    }else if(encoder < 5.102787)
    {
        angle_data.data =  foward_position;
    }else if(encoder == 5.102787)
    {
        angle_data.data = foward_position;
    }
        angle_data.data = foward_position;
      //RCLCPP_INFO(this->get_logger(), "encoder %f ecd %f position %f foward %f", encoder,ecd,position,foward_position);
      angle_pid_set_pub->publish(angle_data);
}
void ChassisTeleop::rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg) {
    if (!this->cmd_publisher->is_activated()) return;
    //RCLCPP_INFO(this->get_logger(), "encoder %f position %f", encoder,position);
    RC_control = *msg;
    double vx_set_control = 0,vy_set_control = 0,az_set_control = 0;
    double vx_set = 0, vy_set = 0, az_set = 0;
    double sin_yaw = 0, cos_yaw = 0;
    double vx_filter_output = 0,vy_filter_output = 0;
    vx_set_control = RC_control.ch_left_y * x_max_speed;
    vy_set_control = RC_control.ch_left_x * y_max_speed;
    az_set_control = -RC_control.ch_wheel * rotate_max_speed;
    //键盘控制
    if(RC_control.key_w)
    {
        vx_set_control = x_max_speed;
    }else if(RC_control.key_s)
    {
        vx_set_control = -x_max_speed;
    }else if(RC_control.key_a)
    {
        vy_set_control = y_max_speed;
    }else if(RC_control.key_d)
    {
        vy_set_control = -y_max_speed;
    }

    if (vx_set_control == 0) x_filter->reset();
    if (vy_set_control == 0) y_filter->reset();
    vx_filter_output = x_filter->first_order_filter(vx_set_control);
    vy_filter_output = y_filter->first_order_filter(vy_set_control);

    //遥控器控制
    if (RC_control.sw_right == gary_msgs::msg::DR16Receiver::SW_DOWN) {
        return;
    }
       //不跟随云台
    else if (RC_control.sw_right == gary_msgs::msg::DR16Receiver::SW_UP) {
        az_set = az_set_control;
        twist.linear.x = vx_filter_output;
        twist.linear.y = -vy_filter_output;
        twist.angular.z = az_set;
    }
     //跟随云台
      else if (RC_control.sw_right == gary_msgs::msg::DR16Receiver::SW_MID) {
          sin_yaw = 0, cos_yaw = 0;
          sin_yaw = sin(-gary_chassis::yaw.relative_angle);
          cos_yaw = cos(-gary_chassis::yaw.relative_angle);
          vx_set = cos_yaw * vy_filter_output + sin_yaw * vx_filter_output;
          vy_set = -sin_yaw * vy_filter_output + cos_yaw * vx_filter_output;
          az_set = angle_follow_pid.out;
          twist.linear.x = vx_set;
          twist.linear.y = vy_set;
          twist.angular.z = az_set;
      }
        //swing
/*    else if(RC_control.sw_right == gary_msgs::msg::DR16Receiver::SW_UP)
    {
        sin_yaw = sin(-gary_chassis::yaw.relative_angle);
        cos_yaw = cos(-gary_chassis::yaw.relative_angle);
        vx_set = cos_yaw * vy_filter_output + sin_yaw * vx_filter_output;
        vy_set = -sin_yaw * vy_filter_output + cos_yaw * vx_filter_output;
        az_set = rotate_max_speed;
        twist.linear.x = vx_set;
        twist.linear.y = vy_set;
        twist.angular.z = az_set;
    }*/

    cmd_publisher->publish(twist);
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

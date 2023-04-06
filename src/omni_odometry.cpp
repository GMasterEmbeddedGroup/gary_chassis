#include "gary_chsssis/omni_odometry.hpp"

using namespace std::chrono_literals;

using namespace gary_chassis;


OmniOdom::OmniOdom(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("omni_odom",
                                                                                                   options) {
    //declare params
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("joint_topic", "/dynamic_joint_states");
    this->declare_parameter("use_break", true);
    this->declare_parameter("update_rate", 200.0f);
    this->declare_parameter("a",0.24492);
    this->declare_parameter("b",0.24492);
    this->declare_parameter("r",0.0763);
}

CallbackReturn OmniOdom::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);


    //get joint_topic
    this->joint_topic = this->get_parameter("joint_topic").as_string();
    this->joint_subscriber = this->create_subscription<control_msgs::msg::DynamicJointState>(
            this->joint_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&OmniOdom::joint_callback, this, std::placeholders::_1));
    this->joint_state_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

    //get
    //get odom_topic
    this->omni_odom_topic = this->get_parameter("odom_topic").as_string();
    this->omni_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(this->omni_odom_topic,
                                                                                rclcpp::SystemDefaultsQoS());

    //get use_break
    this->use_break = this->get_parameter("use_break").as_bool();

    //get update_rate
    this->update_rate = this->get_parameter("update_rate").as_double();

    this->a = this->get_parameter("a").as_double();
    this->b = this->get_parameter("b").as_double();
    this->r = this->get_parameter("r").as_double();

    this->omni_kinematics = std::make_shared<gary_chassis::OmniKinematics>(this->a, this->b, this->r);

    RCLCPP_INFO(this->get_logger(), "configured");

    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniOdom::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    this->omni_odom_publisher.reset();
    this->joint_subscriber.reset();
    this->omni_kinematics.reset();

    RCLCPP_INFO(this->get_logger(), "cleaning up");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniOdom::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //activate lifecycle publisher
    this->omni_odom_publisher->on_activate();
    this->omni_last_time = this->get_clock()->now();
    //create timer
    this->timer_update = this->create_wall_timer(1000ms / this->update_rate, [this] { update(); });

    RCLCPP_INFO(this->get_logger(), "activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniOdom::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //deactivate lifecycle publisher
    this->omni_odom_publisher->on_deactivate();

    //destroy timer
    this->timer_update.reset();

    RCLCPP_INFO(this->get_logger(), "deactivated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniOdom::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->omni_odom_publisher.get() != nullptr) this->omni_odom_publisher.reset();
    if (this->joint_subscriber.get() != nullptr) this->joint_subscriber.reset();
    if (this->timer_update != nullptr) this->timer_update->reset();
    if (this->omni_kinematics != nullptr) this->omni_kinematics.reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniOdom::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->omni_odom_publisher.get() != nullptr) this->omni_odom_publisher.reset();
    if (this->joint_subscriber.get() != nullptr) this->joint_subscriber.reset();
    if (this->timer_update != nullptr) this->timer_update->reset();
    if (this->omni_kinematics != nullptr) this->omni_kinematics.reset();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}


void OmniOdom::joint_callback(control_msgs::msg::DynamicJointState::SharedPtr msg) {
    this->joint_state = *msg;
    this->joint_state_timestamp = this->get_clock()->now();
}

void OmniOdom::update() {

    rclcpp::Time time_now = this->get_clock()->now();

    bool joint_state_available = (time_now - this->joint_state_timestamp).seconds() <= 0.5;
    double lb_speed;
    double lf_speed;
    double rb_speed;
    double rf_speed;
    if (joint_state_available) {
        bool offline = true;
        //get four motors' velocity
        for (unsigned long i = 0; i < this->joint_state.joint_names.size(); ++i) {
            if (this->joint_state.joint_names[i] == "chassis_left_front") {
                for (unsigned long j = 0; j < this->joint_state.interface_values[i].interface_names.size(); ++j) {

                    if (this->joint_state.interface_values[i].interface_names[j] == "offline" && this->joint_state.interface_values[i].values[j] == 0.0f) {
                        offline = false;
                    }

                    if (this->joint_state.interface_values[i].interface_names[j] == "velocity") {
                        lf_speed = this->joint_state.interface_values[i].values[j];

                    }
                }
            }
        }
        for (unsigned long i = 0; i < this->joint_state.joint_names.size(); ++i) {
            if (this->joint_state.joint_names[i] == "chassis_left_back") {
                for (unsigned long j = 0; j < this->joint_state.interface_values[i].interface_names.size(); ++j) {

                    if (this->joint_state.interface_values[i].interface_names[j] == "offline" && this->joint_state.interface_values[i].values[j] == 0.0f) {
                        offline = false;
                    }

                    if (this->joint_state.interface_values[i].interface_names[j] == "velocity") {
                        lb_speed = this->joint_state.interface_values[i].values[j];

                    }
                }
            }
        }
        for (unsigned long i = 0; i < this->joint_state.joint_names.size(); ++i) {
            if (this->joint_state.joint_names[i] == "chassis_right_front") {
                for (unsigned long j = 0; j < this->joint_state.interface_values[i].interface_names.size(); ++j) {

                    if (this->joint_state.interface_values[i].interface_names[j] == "offline" && this->joint_state.interface_values[i].values[j] == 0.0f) {
                        offline = false;
                    }

                    if (this->joint_state.interface_values[i].interface_names[j] == "velocity") {
                        rf_speed = this->joint_state.interface_values[i].values[j];

                    }
                }
            }
        }
        for (unsigned long i = 0; i < this->joint_state.joint_names.size(); ++i) {
            if (this->joint_state.joint_names[i] == "chassis_right_back") {
                for (unsigned long j = 0; j < this->joint_state.interface_values[i].interface_names.size(); ++j) {

                    if (this->joint_state.interface_values[i].interface_names[j] == "offline" && this->joint_state.interface_values[i].values[j] == 0.0f) {
                        offline = false;
                    }

                    if (this->joint_state.interface_values[i].interface_names[j] == "velocity") {
                        rb_speed = this->joint_state.interface_values[i].values[j];

                    }
                }
            }
        }
    }
    std::map<std::string, double> chassis_speed_odom;
    std::map<std::string, double> wheel_speed_odom;
    wheel_speed_odom.emplace("left_back",lb_speed);
    wheel_speed_odom.emplace("right_back",rb_speed);
    wheel_speed_odom.emplace("left_front",lf_speed);
    wheel_speed_odom.emplace("right_front",rf_speed);

    //odometry calculate
    chassis_speed_odom = omni_kinematics->forward_solve(wheel_speed_odom,WHEEL_OFFLINE_NONE);

    nav_msgs::msg::Odometry omni_odom_data;
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

    this->omni_odom_publisher->publish(omni_odom_data);
    omni_last_time = omni_current_time;
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr <OmniOdom> omni_odom = std::make_shared<OmniOdom>(rclcpp::NodeOptions());

    exe.add_node(omni_odom->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_chassis::OmniOdom)


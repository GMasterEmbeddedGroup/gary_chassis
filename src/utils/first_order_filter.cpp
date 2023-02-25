#include "utils/first_order_filter.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace gary_chassis;

First_orderFilter::First_orderFilter(double max_accel) {
    this->max_accel = max_accel;
    this->last_time = rclcpp::Clock{RCL_SYSTEM_TIME}.now().seconds();
    this->out = 0;
}

double First_orderFilter::first_order_filter(double input) {
    double time_now = rclcpp::Clock{RCL_SYSTEM_TIME}.now().seconds();
    double delta_time = time_now - this->last_time;
    this->last_time = time_now;
    double delta_accel = this->max_accel * delta_time;
    double delta_input = input - this->out;
    if (delta_input > delta_accel) delta_input = delta_accel;
    if (delta_input < - delta_accel) delta_input = - delta_accel;
    this->out += delta_input;
    return this->out;
}

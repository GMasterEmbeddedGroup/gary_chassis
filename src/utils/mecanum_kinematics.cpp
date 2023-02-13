#include "utils/mecanum_kinematics.hpp"

using namespace gary_chassis;

MecanumKinematics::MecanumKinematics(double a, double b, double r) {
    this->a = a;
    this->b = b;
    this->r = r;
}

std::map<std::string, double> MecanumKinematics::forward_solve(const std::map<std::string, double>& wheel_speed) {
    std::map<std::string, double> chassis_speed;

    double lf = wheel_speed.at("left_front");
    double lb = wheel_speed.at("left_back");
    double rf = wheel_speed.at("right_front");
    double rb = wheel_speed.at("right_back");

    double vx = (-rf + lf + lb - rb) / 4 * this->r;
    double vy = (-rf - lf + lb + rb) / 4 * this->r;
    double az = (-rf - lf - lb - rb) / 4 / (this->a + this->b);

    chassis_speed.insert(std::make_pair("vx", vx));
    chassis_speed.insert(std::make_pair("vy", vy));
    chassis_speed.insert(std::make_pair("az", az));

    return chassis_speed;
}

std::map<std::string, double> MecanumKinematics::inverse_solve(const std::map<std::string, double>& chassis_speed) {
    std::map<std::string, double> wheel_speed;

    double vx = chassis_speed.at("vx");
    double vy = chassis_speed.at("vy");
    double az = chassis_speed.at("az");

    double rf = (+ vx - vy + az * (a + b)) / this->r;
    double lf = (+ vx + vy + az * (a + b)) / this->r;
    double lb = (- vx + vy + az * (a + b)) / this->r;
    double rb = (- vx - vy + az * (a + b)) / this->r;

    wheel_speed.insert(std::make_pair("right_front", rf));
    wheel_speed.insert(std::make_pair("left_front", lf));
    wheel_speed.insert(std::make_pair("left_back", lb));
    wheel_speed.insert(std::make_pair("right_back", rb));

    return wheel_speed;
}
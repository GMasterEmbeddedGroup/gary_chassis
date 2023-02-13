#pragma once

#include <map>

namespace gary_chassis{

class MecanumKinematics {
public:
    MecanumKinematics(double a, double b, double r);

    std::map<std::string, double> forward_solve(const std::map<std::string, double>& wheel_speed);
    std::map<std::string, double> inverse_solve(const std::map<std::string, double>& chassis_speed);

private:
    double a;
    double b;
    double r;

};
}
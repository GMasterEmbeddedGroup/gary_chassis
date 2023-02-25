#pragma once

#include <chrono>

namespace gary_chassis{

    class First_orderFilter {
    public:
        explicit First_orderFilter(double max_accel);
        double first_order_filter(double input);
        void reset();
    private:
        double max_accel;
        double out;
        double last_time;
    };
}
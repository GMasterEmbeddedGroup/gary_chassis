#include "utils/first_order_filter.hpp"

using namespace gary_chassis;

First_orderFilter::First_orderFilter(double p, double frame_period) {
    this->p = p;
    this->frame_period = frame_period;
}

double First_orderFilter::first_order_filter(double input) {
    double out = 0;
    out = p/(p+frame_period)*out + frame_period/(p+frame_period)*input;
    return out;
}


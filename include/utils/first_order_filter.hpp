#pragma once


namespace gary_chassis{


    class First_orderFilter
    {
    public: First_orderFilter(double p, double frame_period);
            double first_order_filter(double input);
    private:
        double p;
        double frame_period;
    };
}
#ifndef __BRAKE_RESISTOR_HPP
#define __BRAKE_RESISTOR_HPP

#include <interfaces/pwm_output_group.hpp>

class BrakeResistor {
public:
    BrakeResistor(PwmOutputGroup<1>& pwm_output) : pwm_output_(pwm_output) {}

    void arm();
    void disarm();
    void update();
    PwmOutputGroup<1>::on_update_result_t on_update(timestamp_t);
    void on_stopped();

    PwmOutputGroup<1>& pwm_output_;

    bool is_armed_ = false;
    bool is_saturated_ = false;
    PwmOutputGroup<1>::on_update_result_t brake_duty_ = std::nullopt;
};

#endif // __BRAKE_RESISTOR_HPP

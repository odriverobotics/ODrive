
#include "brake_resistor.hpp"
#include "odrive_main.h"

void BrakeResistor::arm() {
    is_armed_ = true;
    pwm_output_.start(MEMBER_CB(this, on_update), MEMBER_CB(this, on_stopped));
}

void BrakeResistor::disarm() {
    pwm_output_.stop();
}

void BrakeResistor::update() {
    // TODO: Currently this is separate from on_update() because on_update only
    // runs if the brake resistor is enabled.
    // In the future part of this update function should be moved to a system-
    // wide power solver.
    // The power solver would collect constraints from all components (incl
    // brake resistor) and finally inform the brake resistor what amount of
    // power to dump.

    float Ibus_sum = 0.0f;

    for (Axis& axis: axes) {
        if (axis.motor_.is_armed_) {
            Ibus_sum += axis.motor_.I_bus_;
        }
    }

    float brake_duty;

    if (odrv.config_.enable_brake_resistor) {
        if (!(odrv.config_.brake_resistance > 0.0f)) {
            odrv.disarm_with_error(ODrive::ERROR_INVALID_BRAKE_RESISTANCE);
            brake_duty_ = std::nullopt;
            return;
        }
    
        // Don't start braking until -Ibus > regen_current_allowed
        float brake_current = -Ibus_sum - odrv.config_.max_regen_current;
        brake_duty = brake_current * odrv.config_.brake_resistance / vbus_voltage;
        
        if (odrv.config_.enable_dc_bus_overvoltage_ramp && (odrv.config_.brake_resistance > 0.0f) && (odrv.config_.dc_bus_overvoltage_ramp_start < odrv.config_.dc_bus_overvoltage_ramp_end)) {
            brake_duty += std::max((vbus_voltage - odrv.config_.dc_bus_overvoltage_ramp_start) / (odrv.config_.dc_bus_overvoltage_ramp_end - odrv.config_.dc_bus_overvoltage_ramp_start), 0.0f);
        }

        if (is_nan(brake_duty)) {
            // Shuts off all motors AND brake resistor, sets error code on all motors.
            odrv.disarm_with_error(ODrive::ERROR_BRAKE_DUTY_CYCLE_NAN);
            brake_duty_ = std::nullopt;
            return;
        }

        if (brake_duty >= 0.95f) {
            is_saturated_ = true;
        }

        // Duty limit at 95% to allow bootstrap caps to charge
        brake_duty = std::clamp(brake_duty, 0.0f, 0.95f);

        // This cannot result in NaN (safe for race conditions) because we check
        // brake_resistance != 0 further up.
        Ibus_sum += brake_duty * vbus_voltage / odrv.config_.brake_resistance;
    } else {
        brake_duty = 0;
    }

    odrv.ibus_ += odrv.ibus_report_filter_k_ * (Ibus_sum - odrv.ibus_);

    if (Ibus_sum > odrv.config_.dc_max_positive_current) {
        odrv.disarm_with_error(ODrive::ERROR_DC_BUS_OVER_CURRENT);
        brake_duty_ = std::nullopt;
        return;
    }
    if (Ibus_sum < odrv.config_.dc_max_negative_current) {
        odrv.disarm_with_error(ODrive::ERROR_DC_BUS_OVER_REGEN_CURRENT);
        brake_duty_ = std::nullopt;
        return;
    }

    brake_duty_ = {{brake_duty}};
}

// @brief Sums up the Ibus contribution of each motor and updates the
// brake resistor PWM accordingly.
PwmOutputGroup<1>::on_update_result_t BrakeResistor::on_update(timestamp_t) {
    return brake_duty_;
}

void BrakeResistor::on_stopped() {
    is_armed_ = false;

    // If the brake resistor gets disarmed even though the user intended to have
    // it enabled it's a system level error.
    if (odrv.config_.enable_brake_resistor) {
        for (auto& axis: axes) {
            axis.motor_.disarm_with_error(Motor::ERROR_SYSTEM_LEVEL);
        }
    }
}

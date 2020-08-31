
#include "open_loop_controller.hpp"
#include <board.h>

void OpenLoopController::update(uint32_t timestamp) {
    if (std::isnan(Id_setpoint_) || std::isnan(Id_setpoint_) || std::isnan(phase_) || std::isnan(phase_vel_)) {
        Id_setpoint_ = 0.0f;
        Iq_setpoint_ = 0.0f;
        Vd_setpoint_ = 0.0f;
        Vq_setpoint_ = 0.0f;
        phase_ = 0.0f;
        phase_vel_ = 0.0f;
        timestamp_ = timestamp;
    }

    float dt = (float)(timestamp - timestamp_) / (float)TIM_1_8_CLOCK_HZ;
    
    Id_setpoint_ = std::clamp(target_current_, Id_setpoint_ - max_current_ramp_ * dt, Id_setpoint_ + max_current_ramp_ * dt);
    Iq_setpoint_ = 0.0f;
    Vd_setpoint_ = std::clamp(target_voltage_, Vd_setpoint_ - max_voltage_ramp_ * dt, Vd_setpoint_ + max_voltage_ramp_ * dt);
    Vq_setpoint_ = 0.0f;

    phase_vel_ = std::clamp(target_vel_, phase_vel_ - max_phase_vel_ramp_ * dt, phase_vel_ + max_phase_vel_ramp_ * dt);
    phase_ = wrap_pm_pi(phase_ + phase_vel_ * dt);
    total_distance_ += phase_vel_ * dt;
    timestamp_ = timestamp;
}

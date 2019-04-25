
#include "odrive_main.h"

#include <math.h>

bool AsyncEstimator::reset() {
    rotor_flux_d_ = 0.0f;
    rotor_flux_q_ = 0.0f;
    rotor_flux_d2_ = 0.0f;
    rotor_flux_q2_ = 0.0f;
    rotor_flux_alpha_ = 0.0f;
    rotor_flux_beta_ = 0.0f;
    phase_ = 0.0f;
    phase_vel_ = 0.0f;
    error_ = ERROR_NONE;
    return true;
}

bool AsyncEstimator::update(float dt) {
    Motor::CurrentControl_t& ictrl = axis_->motor_.current_control_;

    if (axis_->encoder_.error_ != Encoder::ERROR_NONE) {
        error_ |= ERROR_ENCODER_FAILED;
        return false;
    }

    if (!axis_->motor_.config_.async_calibrated) {
        error_ |= ERROR_NOT_CALIBRATED;
        return false;
    }

    float tau_r = axis_->motor_.config_.phase_inductance / axis_->motor_.config_.phase_resistance;
    float l_m = axis_->motor_.config_.mutual_inductance;
    if (!(tau_r > dt)) {
        error_ |= ERROR_UNSTABLE;
        return false;
    }

    float omega_rotor = 2 * M_PI * axis_->encoder_.vel_estimate_ / axis_->encoder_.config_.cpr;
    float omega_slip = ictrl.phase_vel - omega_rotor;

    float drotor_flux_d_dt = (l_m * ictrl.Id_measured - rotor_flux_d_) / tau_r + omega_slip * rotor_flux_q_;
    float drotor_flux_q_dt = (l_m * ictrl.Iq_measured - rotor_flux_q_) / tau_r - omega_slip * rotor_flux_d_;

    float I_alpha_measured = axis_->motor_.I_alpha_beta_measured_[0];
    float I_beta_measured = axis_->motor_.I_alpha_beta_measured_[1];
    float drotor_flux_alpha_dt = (l_m * I_alpha_measured - rotor_flux_alpha_) / tau_r - omega_rotor * rotor_flux_beta_;
    float drotor_flux_beta_dt = (l_m * I_beta_measured - rotor_flux_beta_) / tau_r + omega_rotor * rotor_flux_alpha_;
    
    rotor_flux_d_ += drotor_flux_d_dt * dt;
    rotor_flux_q_ += drotor_flux_q_dt * dt;

    float new_rotor_flux_alpha_ = rotor_flux_alpha_ + drotor_flux_alpha_dt * dt;
    float new_rotor_flux_beta_ = rotor_flux_beta_ + drotor_flux_beta_dt * dt;

    //float cross_product = new_rotor_flux_beta_ * rotor_flux_alpha_ - new_rotor_flux_alpha_ * rotor_flux_beta_;
    //float norm = new_rotor_flux_alpha_ * new_rotor_flux_alpha_ + new_rotor_flux_beta_ * new_rotor_flux_beta_;
    //float omega_rotor_flux = cross_product / (norm * dt + config_.small_number);
    //phase_vel_ += dt * norm * vel_kp_ * (omega_rotor_flux - phase_vel_);

    rotor_flux_alpha_ = new_rotor_flux_alpha_;
    rotor_flux_beta_ = new_rotor_flux_beta_;


    float c_I = our_arm_cos_f32(ictrl.phase);
    float s_I = our_arm_sin_f32(ictrl.phase);
    rotor_flux_d2_ = c_I * rotor_flux_alpha_ + s_I * rotor_flux_beta_;
    rotor_flux_q2_ = c_I * rotor_flux_beta_ - s_I * rotor_flux_alpha_;

    float old_phase = phase_;
    phase_ = fast_atan2(rotor_flux_beta_, rotor_flux_alpha_); // rotor flux angle
    phase_vel_ += vel_kp_ * wrap_pm_pi(phase_ - old_phase) - dt * vel_kp_ * phase_vel_;
    
    //phase_ = fast_atan2(rotor_flux_alpha_, -rotor_flux_beta_); 

    return true;
};

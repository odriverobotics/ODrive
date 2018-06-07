
#include "odrive_main.h"

SensorlessEstimator::SensorlessEstimator()
{
    // Calculate pll gains
    // This calculation is currently identical to the PLL in Encoder
    float pll_bandwidth = 1000.0f;  // [rad/s]
    pll_kp_ = 2.0f * pll_bandwidth;

    // Critically damped
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_);
}

bool SensorlessEstimator::update() {
    // Algorithm based on paper: Sensorless Control of Surface-Mount Permanent-Magnet Synchronous Motors Based on a Nonlinear Observer
    // http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
    // In particular, equation 8 (and by extension eqn 4 and 6).

    // The V_alpha_beta applied immedietly prior to the current measurement associated with this cycle
    // is the one computed two cycles ago. To get the correct measurement, it was stored twice:
    // once by final_v_alpha/final_v_beta in the current control reporting, and once by V_alpha_beta_memory.

    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        error_ |= ERROR_UNSTABLE_GAIN;
        return false;
    }

    // Clarke transform
    float I_alpha_beta[2] = {
        -axis_->motor_.current_meas_.phB - axis_->motor_.current_meas_.phC,
        one_by_sqrt3 * (axis_->motor_.current_meas_.phB - axis_->motor_.current_meas_.phC)};

    // Swap sign of I_beta if motor is reversed
    I_alpha_beta[1] *= axis_->motor_.config_.direction;

    // alpha-beta vector operations
    float eta[2];
    for (int i = 0; i <= 1; ++i) {
        // y is the total flux-driving voltage (see paper eqn 4)
        float y = -axis_->motor_.config_.phase_resistance * I_alpha_beta[i] + V_alpha_beta_memory_[i];
        // flux dynamics (prediction)
        float x_dot = y;
        // integrate prediction to current timestep
        flux_state_[i] += x_dot * current_meas_period;

        // eta is the estimated permanent magnet flux (see paper eqn 6)
        eta[i] = flux_state_[i] - axis_->motor_.config_.phase_inductance * I_alpha_beta[i];
    }

    // Non-linear observer (see paper eqn 8):
    float pm_flux_sqr = pm_flux_linkage_ * pm_flux_linkage_;
    float est_pm_flux_sqr = eta[0] * eta[0] + eta[1] * eta[1];
    float bandwidth_factor = 1.0f / pm_flux_sqr;
    float eta_factor = 0.5f * (observer_gain_ * bandwidth_factor) * (pm_flux_sqr - est_pm_flux_sqr);

    static float eta_factor_avg_test = 0.0f;
    eta_factor_avg_test += 0.001f * (eta_factor - eta_factor_avg_test);

    // alpha-beta vector operations
    for (int i = 0; i <= 1; ++i) {
        // add observer action to flux estimate dynamics
        float x_dot = eta_factor * eta[i];
        // convert action to discrete-time
        flux_state_[i] += x_dot * current_meas_period;
        // update new eta
        eta[i] = flux_state_[i] - axis_->motor_.config_.phase_inductance * I_alpha_beta[i];
    }

    // Flux state estimation done, store V_alpha_beta for next timestep
    V_alpha_beta_memory_[0] = axis_->motor_.current_control_.final_v_alpha;
    V_alpha_beta_memory_[1] = axis_->motor_.current_control_.final_v_beta * axis_->motor_.config_.direction;

    // PLL
    // TODO: the PLL part has some code duplication with the encoder PLL
    // predict PLL phase with velocity
    pll_pos_ = wrap_pm_pi(pll_pos_ + current_meas_period * pll_vel_);
    // update PLL phase with observer permanent magnet phase
    phase_ = fast_atan2(eta[1], eta[0]);
    float delta_phase = wrap_pm_pi(phase_ - pll_pos_);
    pll_pos_ = wrap_pm_pi(pll_pos_ + current_meas_period * pll_kp_ * delta_phase);
    // update PLL velocity
    pll_vel_ += current_meas_period * pll_ki_ * delta_phase;

    return true;
};

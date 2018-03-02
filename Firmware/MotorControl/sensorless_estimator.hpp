#ifndef __SENSORLESS_ESTIMATOR_HPP
#define __SENSORLESS_ESTIMATOR_HPP

class SensorlessEstimator {
public:
    SensorlessEstimator();

    bool update(float* pos_estimate, float* vel_estimate, float* phase);

    Axis* axis = nullptr; // set by Axis constructor

    float phase = 0.0f;                        // [rad]
    float pll_pos = 0.0f;                      // [rad]
    float pll_vel = 0.0f;                      // [rad/s]
    float pll_kp = 0.0f;                       // [rad/s / rad]
    float pll_ki = 0.0f;                       // [(rad/s^2) / rad]
    float observer_gain = 1000.0f;             // [rad/s]
    float flux_state[2] = {0.0f, 0.0f};        // [Vs]
    float V_alpha_beta_memory[2] = {0.0f, 0.0f}; // [V]
    float pm_flux_linkage = 1.58e-3f;          // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
    bool estimator_good = false;
};

#endif /* __SENSORLESS_ESTIMATOR_HPP */

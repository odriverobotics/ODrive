#ifndef __SENSORLESS_ESTIMATOR_HPP
#define __SENSORLESS_ESTIMATOR_HPP

class SensorlessEstimator : public ODriveIntf::SensorlessEstimatorIntf {
public:
    struct Config_t {
        float observer_gain = 1000.0f; // [rad/s]
        float pll_bandwidth = 1000.0f;  // [rad/s]
        float pm_flux_linkage = 1.58e-3f; // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
    };

    bool update();

    Axis* axis_ = nullptr; // set by Axis constructor
    Config_t config_;

    // TODO: expose on protocol
    Error error_ = ERROR_NONE;
    float phase_ = 0.0f;                        // [rad]
    float pll_pos_ = 0.0f;                      // [rad]
    float phase_vel_ = 0.0f;                    // [rad/s]
    float vel_estimate_ = 0.0f;                 // [turns/s]
    // float pll_kp_ = 0.0f;                       // [rad/s / rad]
    // float pll_ki_ = 0.0f;                       // [(rad/s^2) / rad]
    float flux_state_[2] = {0.0f, 0.0f};        // [Vs]
    float V_alpha_beta_memory_[2] = {0.0f, 0.0f}; // [V]
    bool estimator_good_ = false;
};

#endif /* __SENSORLESS_ESTIMATOR_HPP */

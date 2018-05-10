#ifndef __SENSORLESS_ESTIMATOR_HPP
#define __SENSORLESS_ESTIMATOR_HPP

class SensorlessEstimator {
public:
    enum Error_t {
        ERROR_NONE = 0,
        ERROR_UNSTABLE_GAIN = 0x01,
    };

    SensorlessEstimator();

    bool update();

    Axis* axis_ = nullptr; // set by Axis constructor

    // TODO: expose on protocol
    Error_t error_ = ERROR_NONE;
    float phase_ = 0.0f;                        // [rad]
    float pll_pos_ = 0.0f;                      // [rad]
    float pll_vel_ = 0.0f;                      // [rad/s]
    float pll_kp_ = 0.0f;                       // [rad/s / rad]
    float pll_ki_ = 0.0f;                       // [(rad/s^2) / rad]
    float observer_gain_ = 1000.0f;             // [rad/s]
    float flux_state_[2] = {0.0f, 0.0f};        // [Vs]
    float V_alpha_beta_memory_[2] = {0.0f, 0.0f}; // [V]
    float pm_flux_linkage_ = 1.58e-3f;          // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
    bool estimator_good_ = false;

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("error", &error_),
            make_protocol_property("phase", &phase_),
            make_protocol_property("pll_pos", &pll_pos_),
            make_protocol_property("pll_vel", &pll_vel_),
            make_protocol_property("pll_kp", &pll_kp_),
            make_protocol_property("pll_ki", &pll_ki_)
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(SensorlessEstimator::Error_t)

#endif /* __SENSORLESS_ESTIMATOR_HPP */

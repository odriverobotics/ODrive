#ifndef __SENSORLESS_ESTIMATOR_HPP
#define __SENSORLESS_ESTIMATOR_HPP

class SensorlessEstimator {
public:
    enum Error_t {
        ERROR_NONE = 0,
        ERROR_UNSTABLE_GAIN = 0x01,
    };

    struct Config_t {
        float observer_gain = 1000.0f; // [rad/s]
        float pll_bandwidth = 1000.0f;  // [rad/s]
        float pm_flux_linkage = 1.58e-3f; // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
    };

    explicit SensorlessEstimator(Config_t& config);

    bool update();

    Axis* axis_ = nullptr; // set by Axis constructor
    Config_t& config_;

    // TODO: expose on protocol
    Error_t error_ = ERROR_NONE;
    float phase_ = 0.0f;                        // [rad]
    float pll_pos_ = 0.0f;                      // [rad]
    float vel_estimate_ = 0.0f;                      // [rad/s]
    // float pll_kp_ = 0.0f;                       // [rad/s / rad]
    // float pll_ki_ = 0.0f;                       // [(rad/s^2) / rad]
    float flux_state_[2] = {0.0f, 0.0f};        // [Vs]
    float V_alpha_beta_memory_[2] = {0.0f, 0.0f}; // [V]
    bool estimator_good_ = false;

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("error", &error_),
            make_protocol_property("phase", &phase_),
            make_protocol_property("pll_pos", &pll_pos_),
            make_protocol_property("vel_estimate", &vel_estimate_),
            // make_protocol_property("pll_kp", &pll_kp_),
            // make_protocol_property("pll_ki", &pll_ki_),
            make_protocol_object("config",
                make_protocol_property("observer_gain", &config_.observer_gain),
                make_protocol_property("pll_bandwidth", &config_.pll_bandwidth),
                make_protocol_property("pm_flux_linkage", &config_.pm_flux_linkage)
            )
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(SensorlessEstimator::Error_t)

#endif /* __SENSORLESS_ESTIMATOR_HPP */

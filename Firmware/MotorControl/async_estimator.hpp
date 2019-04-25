#ifndef __ASYNC_ESTIMATOR_HPP
#define __ASYNC_ESTIMATOR_HPP

class AsyncEstimator {
public:
    enum Error_t {
        ERROR_NONE = 0,
        ERROR_NOT_CALIBRATED = 0x01,
        ERROR_ENCODER_FAILED = 0x02,
        ERROR_UNSTABLE = 0x04,
    };

    struct Config_t {
        float small_number = 0.0f;
    };

    explicit AsyncEstimator(Config_t& config) : config_(config) {}

    bool init() { return true; }

    bool reset();
    bool update(float dt);

    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    // TODO: expose on protocol
    Error_t error_ = ERROR_NONE;
    float rotor_flux_d_ = 0.0f;        // [Vs]
    float rotor_flux_q_ = 0.0f;        // [Vs]
    float rotor_flux_alpha_ = 0.0f;        // [Vs]
    float rotor_flux_beta_ = 0.0f;        // [Vs]
    float rotor_flux_d2_ = 0.0f;        // [Vs]
    float rotor_flux_q2_ = 0.0f;        // [Vs]
    float phase_ = 0.0f;        // [rad]
    float phase_vel_ = 0.0f;        // [rad/s]
    float vel_kp_ = 200.0f;        // [rad/s]

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("error", &error_),
            make_protocol_property("rotor_flux_d", &rotor_flux_d_),
            make_protocol_property("rotor_flux_q", &rotor_flux_q_),
            make_protocol_property("rotor_flux_alpha", &rotor_flux_alpha_),
            make_protocol_property("rotor_flux_beta", &rotor_flux_beta_),
            make_protocol_property("rotor_flux_d2", &rotor_flux_d2_),
            make_protocol_property("rotor_flux_q2", &rotor_flux_q2_),
            make_protocol_property("phase", &phase_),
            make_protocol_property("phase_vel", &phase_vel_),
            make_protocol_property("vel_kp_", &vel_kp_),
            make_protocol_function("reset", *this, &AsyncEstimator::reset),
            make_protocol_object("config",
                make_protocol_property("small_number", &config_.small_number)
            )
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(AsyncEstimator::Error_t)

#endif /* __ASYNC_ESTIMATOR_HPP */

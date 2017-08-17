#pragma once

typedef struct {
    float phase;
    float pll_pos;
    float pll_vel;
    float pll_kp;
    float pll_ki;
    float observer_gain; // [rad/s]
    float flux_state[2]; // [Vs]
    float V_alpha_beta_memory[2]; // [V]
    float pm_flux_linkage; // [V / (rad/s)]
    bool estimator_good;
    float spin_up_current; // [A]
    float spin_up_acceleration; // [rad/s^2]
    float spin_up_target_vel; // [rad/s]
} Sensorless_t;
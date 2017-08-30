#pragma once

class Sensorless{
public:
    Sensorless();

    // TODO:  Get/set functions as needed
    float phase = 0.0f;
    float pll_pos = 0.0f;
    float pll_vel = 0.0f;
    float pll_kp = 0.0f;
    float pll_ki = 0.0f;
    float observer_gain = 1000.0f;           // [rad/s]
    float flux_state[] = {0.0f, 0.0f};           // [Vs]
    float V_alpha_beta_memory[] = {0.0f, 0.0f};  // [V]
    float pm_flux_linkage = 1.538e-3f;         // [V / (rad/s)] { 5.51328895422 / (<pole pairs> * <rpm/v>) }
    bool estimator_good = false;
    float spin_up_current = 10.0f;       // [A]
    float spin_up_acceleration = 400.0f;  // [rad/s^2]
    float spin_up_target_vel = 400.0f;    // [rad/s]

    // TODO:  Functions which operate on Sensorless objects should be here
}


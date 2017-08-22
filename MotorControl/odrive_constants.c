#include <odrive_constants.h>
#include <main.h>

float elec_rad_per_enc = POLE_PAIRS * 2 * M_PI * (1.0f / (float)ENCODER_CPR);
float brake_resistance = BRAKE_RESISTANCE; // [ohm]

static const float current_meas_period = CURRENT_MEAS_PERIOD;
static const int current_meas_hz = CURRENT_MEAS_HZ;

// This value is updated by the DC-bus reading ADC.
// Arbitrary non-zero inital value to avoid division by zero if ADC reading is late
float vbus_voltage = 12.0f;

float get_current_meas_period()
{
    return current_meas_period;
}

float get_current_meas_hz()
{
    return current_meas_hz;
}

float get_vbus_voltage()
{
    return vbus_voltage;
}

const float *get_vbus_voltage_pt()
{
    return &vbus_voltage;
}

void set_vbus_voltage(float fVoltage)
{
    vbus_voltage = fVoltage;
}
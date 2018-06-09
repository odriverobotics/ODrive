
#ifndef ODriveArduino_h
#define ODriveArduino_h

#include "Arduino.h"

class ODriveArduino {
public:
    enum ParamNamesFloat {
        PARAM_FLOAT_POS_SETPOINT,
        PARAM_FLOAT_POS_GAIN,
        PARAM_FLOAT_VEL_SETPOINT,
        PARAM_FLOAT_VEL_GAIN,
        PARAM_FLOAT_VEL_INTEGRATOR_GAIN,
        PARAM_FLOAT_VEL_INTEGRATOR_CURRENT,
        PARAM_FLOAT_VEL_LIMIT,
        PARAM_FLOAT_CURRENT_SETPOINT,
        PARAM_FLOAT_CALIBRATION_CURRENT,
        PARAM_FLOAT_PHASE_INDUCTANCE,
        PARAM_FLOAT_PHASE_RESISTANCE,
        PARAM_FLOAT_CURRENT_MEAS_PHB,
        PARAM_FLOAT_CURRENT_MEAS_PHC,
        PARAM_FLOAT_DC_CALIB_PHB,
        PARAM_FLOAT_DC_CALIB_PHC,
        PARAM_FLOAT_SHUNT_CONDUCTANCE,
        PARAM_FLOAT_PHASE_CURRENT_REV_GAIN,
        PARAM_FLOAT_CURRENT_CONTROL_CURRENT_LIM,
        PARAM_FLOAT_CURRENT_CONTROL_P_GAIN,
        PARAM_FLOAT_CURRENT_CONTROL_I_GAIN,
        PARAM_FLOAT_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_D,
        PARAM_FLOAT_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_Q,
        PARAM_FLOAT_CURRENT_CONTROL_IBUS,
        PARAM_FLOAT_ENCODER_PHASE,
        PARAM_FLOAT_ENCODER_PLL_POS,
        PARAM_FLOAT_ENCODER_PLL_VEL,
        PARAM_FLOAT_ENCODER_PLL_KP,
        PARAM_FLOAT_ENCODER_PLL_KI,
    };

    enum ParamNamesInt32 {
        PARAM_INT_CONTROL_MODE,
        PARAM_INT_ENCODER_ENCODER_OFFSET,
        PARAM_INT_ENCODER_ENCODER_STATE,
        PARAM_INT_ERROR,
    };

    enum ParamNamesBool{
        PARAM_BOOL_THREAD_READY,
        PARAM_BOOL_ENABLE_CONTROL,
        PARAM_BOOL_DO_CALIBRATION,
        PARAM_BOOL_CALIBRATION_OK,
    };

    enum ParamNamesUint16{
        PARAM_UINT16_CONTROL_DEADLINE,
        PARAM_UINT16_LAST_CPU_TIME,
    };

    ODriveArduino(Stream& serial);

    // Get
    float getBusVoltage();

    float    GetParameter(int motor_number, ParamNamesFloat parameter);
    int32_t  GetParameter(int motor_number, ParamNamesInt32 parameter);
    bool     GetParameter(int motor_number, ParamNamesBool parameter);
    uint16_t GetParameter(int motor_number, ParamNamesUint16 parameter);

    // Set
    void SetPosition(int motor_number, float position);
    void SetPosition(int motor_number, float position, float velocity_feedforward);
    void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    void SetVelocity(int motor_number, float velocity);
    void SetVelocity(int motor_number, float velocity, float current_feedforward);

    void SetParameter(int motor_number, ParamNamesFloat parameter, float value);
    void SetParameter(int motor_number, ParamNamesInt32 parameter, int32_t value);
    void SetParameter(int motor_number, ParamNamesBool parameter, bool value);
    void SetParameter(int motor_number, ParamNamesUint16 parameter, uint16_t value);
private:
    float readFloat();
    int32_t readInt();
    String readString();

    Stream& serial_;
};

#endif //ODriveArduino_h
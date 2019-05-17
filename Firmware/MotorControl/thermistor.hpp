#ifndef __THERMISTOR_HPP
#define __THERMISTOR_HPP

#include "utils.h"

class TempSensor_t {
public:
    virtual bool init() = 0;
    virtual bool read_temp(float* temp) = 0;
};

template<typename TADC, TADC& adc>
class Thermistor_t : public TempSensor_t {
public:
    Thermistor_t(const float* coeffs, size_t n_coeffs) :
        coeffs_(coeffs), n_coeffs_(n_coeffs) {}

    bool init() final {
        return adc.init();
    }

    /**
     * @brief Reads the temperature in degrees celsius.
     * Returns true if the function suceeded or false otherwise.
     */
    bool read_temp(float* temp) final {
        float dummy;
        if (!temp)
            temp = &dummy;

        float adc_val;
        if (!adc.get_normalized(&adc_val)) {
            return false;
        }
        *temp = horner_fma(adc_val, coeffs_, n_coeffs_);
        return true;
    }

private:
    const float* coeffs_;
    size_t n_coeffs_;
};

template<typename TADC, TADC& adc>
class Pt100 : public TempSensor_t {
public:
    /**
     * @brief Initializes a Pt100 temperature sensor.
     * 
     * The following voltage divider circuit is assumed:
     * 
     *        V_ref
     *          |
     *  upper_resistance
     *          |
     *          o-----> ADC
     *          |
     *        Pt100
     *          |
     *         GND
     * 
     * @param alpha: The linear coefficient of the sensor. If R_100 is the
     *        resistance at 100°C the value is calculated as
     *          (R_100 - 100Ohm) / (100Ohm * 100°C)
     *        The most common value is 0.003851.
     * @param upper_resistance: The upper resistance of the voltage divider.
     */
    Pt100(float alpha, float upper_resistance)
        : mult_(0.01f / alpha), upper_resistance_(upper_resistance) {}

    bool init() final {
        return adc.init();
    }

    bool read_temp(float* temp) final {
        float dummy;
        if (!temp)
            temp = &dummy;

        float adc_val;
        if (!adc.get_normalized(&adc_val)) {
            return false;
        }
        float resistance = adc_val * upper_resistance_ / (1.0f - adc_val);
        *temp = mult_ * (resistance - 100.0f);
        return true;
    }

private:
    float mult_;
    float upper_resistance_;
};

#endif // __THERMISTOR_HPP
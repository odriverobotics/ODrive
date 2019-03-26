#ifndef __THERMISTOR_HPP
#define __THERMISTOR_HPP

class Thermistor_t {
public:
    Thermistor_t(ADCChannel_t* adc, const float* coeffs, size_t n_coeffs) :
        adc_(adc), coeffs_(coeffs), n_coeffs_(n_coeffs) {}

    bool init() {
        return adc_ && adc_->init();
    }

    /**
     * @brief Reads the temperature in degrees celsius.
     * Returns true if the function suceeded or false otherwise.
     */
    bool read_temp(float* temp) {
        if (temp && adc_) {
            float adc_val;
            if (!adc_->get_normalized(&adc_val)) {
                return false;
            }
            *temp = horner_fma(adc_val, coeffs_, n_coeffs_);
            return true;
        } else {
            return false;
        }
    }

private:
    ADCChannel_t* adc_;
    const float* coeffs_;
    size_t n_coeffs_;
};

#endif // __THERMISTOR_HPP
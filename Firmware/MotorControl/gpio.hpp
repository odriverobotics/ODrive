#ifndef __GPIO_HPP
#define __GPIO_HPP

class GPIO_t {
public:
    virtual ~GPIO_t() = default;

    enum MODE {
        INPUT,
        OUTPUT,
        OPEN_DRAIN,
    };
    enum PULL {
        PULL_UP,
        PULL_DOWN,
        NO_PULL
    };
    enum SPEED {
        SLOW,       // up to 2 MHz
        MEDIUM,     // 12.5 to 50 MHz
        FAST,       // 25 to 100 MHz
        VERY_FAST   // 50 to 200 MHz
    };

    /*
    * @brief Initializes the GPIO.
    */
    virtual bool init(MODE mode, PULL pull, bool state = false) = 0;

    /* @brief Sets the state of the GPIO */
    virtual void write(bool value) = 0;

    /* @brief Reads the current state of the GPIO */
    virtual bool read() = 0;

    /* @brief Subscribes to a rising or falling edge on the pin*/
    virtual bool subscribe(bool rising_edge, bool falling_edge,
        void (*callback)(void*), void* ctx) = 0;

    /** @brief Puts the GPIO into high impedance and removes all subscribers */
    virtual void deinit() = 0;
};

#endif // __GPIO_HPP
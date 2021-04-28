#ifndef __CURRENT_LIMITER_HPP
#define __CURRENT_LIMITER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

class CurrentLimiter {
public:
    virtual ~CurrentLimiter() = default;
    virtual float get_current_limit(float base_current_lim) const = 0;
};

#endif // __CURRENT_LIMITER_HPP

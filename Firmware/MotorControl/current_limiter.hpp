#ifndef __CURRENT_LIMITER_HPP
#define __CURRENT_LIMITER_HPP

class CurrentLimiter {
public:
    virtual ~CurrentLimiter() = default;
    virtual float get_current_limit(float base_current_lim) const = 0;
};

#endif // __CURRENT_LIMITER_HPP

#ifndef __PWM_OUTPUT_HPP
#define __PWM_OUTPUT_HPP

#include <fibre/callback.hpp>
#include <stdint.h>

using timestamp_t = uint32_t;

/**
 * @brief Base class for a group of PWM outputs.
 * 
 * Usage:
 * 
 *  1. At startup the PWM output is off. For PWM outputs that control power FETs
 *     this usually means that the FETs are off and the outputs are floating.
 * 
 *  2. The application starts the PWM output by calling start() and giving it an
 *     on_update callback. This does not yet enable the PWM outputs immediately.
 * 
 *  3. The PWM output implementation periodically requests duty-cycle settings
 *     from the application by calling on_update(). The frequency of these calls
 *     is implementation specific and need not be equal to the PWM frequency.
 *     The application can either bail out (by returning std::nullopt) or return
 *     an array of per-channel-optional duty-cycle values.
 *     If std::nullopt is returned for any of the channels, that channel is
 *     disabled. Otherwise the duty-cycle must be in [0.0, 1.0].
 *     Note that some implementations require either all or none of the channels
 *     to be disabled.
 * 
 *  4. The PWM output stops if any of the following happens:
 *      - on_update() returns std::nullopt
 *      - on_update() returns float values that are not in [0.0, 1.0]
 *      - stop() is called
 */
template<size_t N>
struct PwmOutputGroup {
    using on_update_result_t = std::optional<
        std::array<std::optional<float>, N>
    >;
    using on_update_cb_t = fibre::Callback<on_update_result_t, timestamp_t>;
    using on_stopped_cb_t = fibre::Callback<void>;

    /**
     * @brief Enables the PWM output lazily.
     * 
     * The actual channels are only enabled once a valid duty-cycle is available
     * for them.
     * 
     * The frequency of the PWM is an implementation detail.
     * 
     * @param on_update: Called by PwmOutputGroup to request new duty cycle
     *        settings from the application.
     *        A timestamp is passed to the application that marks the middle of
     *        the time window in which the duty-cycle settings will be active.
     * 
     * @param on_stopped: Called by PwmOutputGroup to indicate that it has
     *        stopped operating.
     */
    virtual void start(on_update_cb_t on_update,
                       fibre::Callback<void> on_stopped) = 0;

    /**
     * @brief Stops the PWM output asynchronously.
     * 
     * The on_stopped callback that was passed to start() is called afterwards.
     */
    virtual void stop() = 0;
};

#endif // __PWM_OUTPUT_HPP

#ifndef __COMPONENT_HPP
#define __COMPONENT_HPP

#include <stdint.h>
#include <optional>
#include <variant>

class ComponentBase {
public:
    /**
     * @brief Shall run the update action of this component.
     * 
     * This function gets called in a low priority interrupt context and is
     * allowed to call CMSIS functions.
     * 
     * @param timestamp: The timestamp (in HCLK ticks) for which this update
     * is run.
     */
    virtual void update(uint32_t timestamp) = 0;
};


template<typename T>
class InputPort;

/**
 * @brief An output port stores a value for consumption by a connecting input
 * port.
 * 
 * Output ports are supposed to be reset at the beginning of a control loop
 * iteration. This ensures that connecting input ports don't use an outdated
 * value and, more importantly, ensures proper handling if the producer of the
 * value is incapable of producing the value for any reason.
 * 
 * Member functions of this class are not thread-safe unless noted otherwise.
 */
template<typename T>
class OutputPort {
public:
    /**
     * @brief Initializes the output port with the specified value.
     * 
     * An initialization value is required for any() to work properly.
     * present() and previous() cannot be used to fetch the
     * initialization value.
     */
    OutputPort(T val) : content_(val) {}
    
    /**
     * @brief Updates the underlying value of this output port.
     */
    void operator=(T value) {
        content_ = value;
        age_ = 0;
    }

    /**
     * @brief Marks the contained value as outdated. The value is not actually
     * deleted and can still be accessed through some of the member functions
     * of this class.
     */
    void reset() {
        // This will eventually overflow to 0 so present() could
        // theoretically return a very old value however it is very likely that
        // the motor will be long disarmed by then.
        age_++;
    }

    /**
     * @brief Returns the value from this control loop iteration or std::nullopt
     * if the value was not yet set during this control loop iteration.
     */
    std::optional<T> present() {
        if (age_ == 0) {
            return content_;
        } else {
            return std::nullopt;
        }
    }

    /**
     * @brief Returns the value from exactly the previous control loop iteration.
     * 
     * If during the last iteration no value was set or the value was already
     * overwritten during this control loop iteration then this function returns
     * std::nullopt.
     */
    std::optional<T> previous() {
        if (age_ == 1) {
            return content_;
        } else {
            return std::nullopt;
        }
    }

    /**
     * @brief Returns the value contained in this output port with disregard of
     * when the value was set.
     * 
     * This function is thread-safe if load/store operations of T are atomic.
     */
    std::optional<T> any() {
        return content_;
    }
    
private:
    uint32_t age_ = 2; // Age in number of control loop iterations
    T content_;
};

/**
 * @brief An input port provides a value from the source to which it's configured.
 * 
 * The source can be one of:
 *  - an internally stored value
 *  - an externally stored value (referenced by a pointer)
 *  - an external OutputPort (referenced by a pointer)
 *  - none (all queries will return std::nullopt)
 * 
 * Member functions of this class are not thread-safe unless otherwise noted.
 */
template<typename T>
class InputPort {
public:
    void connect_to(OutputPort<T>* input_port) {
        content_ = input_port;
    }

    void connect_to(T* input_ptr) {
        content_ = input_ptr;
    }

    void disconnect() {
        content_ = (OutputPort<T>*)nullptr;
    }

    std::optional<T> present() {
        if (content_.index() == 2) {
            OutputPort<T>* ptr = std::get<2>(content_);
            return ptr ? ptr->present() : std::nullopt;
        } else if (content_.index() == 1) {
            T* ptr = std::get<1>(content_);
            return ptr ? std::make_optional(*ptr) : std::nullopt;
        } else {
            return std::get<0>(content_);
        }
    }

    // TODO: probably it makes sense to let the application define that it's
    // ok for this input port to fetch the value from the last iteration.
    // This would provide a general way to resolve same-iteration data path cycles.

    //std::optional<T> previous() {
    //    if (content_.index() == 2) {
    //        OutputPort<T>* ptr = std::get<2>(content_);
    //        return ptr ? ptr->previous() : std::nullopt;
    //    } else if (content_.index() == 1) {
    //        T* ptr = std::get<1>(content_);
    //        return ptr ? std::make_optional(*ptr) : std::nullopt;
    //    } else {
    //        return std::get<0>(content_);
    //    }
    //}

    std::optional<T> any() {
        if (content_.index() == 2) {
            OutputPort<T>* ptr = std::get<2>(content_);
            return ptr ? ptr->any() : std::nullopt;
        } else if (content_.index() == 1) {
            T* ptr = std::get<1>(content_);
            return ptr ? std::make_optional(*ptr) : std::nullopt;
        } else {
            return std::get<0>(content_);
        }
    }
    
private:
    std::variant<T, T*, OutputPort<T>*> content_;
};


#endif // __COMPONENT_HPP
#ifndef __SUBSCRIBER_HPP
#define __SUBSCRIBER_HPP

template<typename ... TArgs>
class Subscriber {
public:
    Subscriber() : callback_(nullptr), ctx_(nullptr) {}

    template<typename TObj, void (TObj::*func)(TArgs...)>
    bool set(TObj& obj) {
        callback_ = nullptr;
        ctx_ = &obj;
        callback_ = [](void* ctx, TArgs ... args){ if (ctx) (((TObj*)ctx)->*func)(args...); };
        return true; // TODO: should we fail if the callback is already set?
    }

    template<typename TObj>
    bool set(void (*func)(TObj*, TArgs...), TObj* obj) {
        callback_ = nullptr;
        ctx_ = obj;
        callback_ = reinterpret_cast<void (*)(void*, TArgs...)>(func);
        return true; // TODO: should we fail if the callback is already set?
    }

    void unset() {
        callback_ = nullptr;
        ctx_ = nullptr;
    }

    void invoke(TArgs ... args) {
        if (callback_) {
            callback_(ctx_, args...);
        }
    }

private:
    void (*callback_)(void*, TArgs...);
    void* ctx_;
};

#endif // __SUBSCRIBER_HPP
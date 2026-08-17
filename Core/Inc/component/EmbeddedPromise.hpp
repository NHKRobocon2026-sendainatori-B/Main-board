/*
 * EmbeddedPromise.hpp
 *
 *  Created on: Aug 17, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_EMBEDDEDPROMISE_HPP_
#define INC_COMPONENT_EMBEDDEDPROMISE_HPP_


#include <memory>

template <typename T>
class Future {
public:
    struct SharedState {
        volatile bool ready = false;
        T value{};
    };

    Future() = default;
    explicit Future(std::shared_ptr<SharedState> state) : state_(state) {}

    bool is_ready() const {
        return state_ && state_->ready;
    }

    bool is_success() const {
        return is_ready() && (state_->value == true);
    }

    bool is_failed() const {
        return is_ready() && (state_->value == false);
    }

    T get() {
        if (!state_) return T{};
        state_->ready = false;
        return state_->value;
    }

    bool valid() const {
        return state_ != nullptr;
    }

private:
    std::shared_ptr<SharedState> state_;
};

template <typename T>
class Promise {
public:
    using SharedState = typename Future<T>::SharedState;

    Promise() : state_(std::make_shared<SharedState>()) {}

    Future<T> get_future() {
        return Future<T>(state_);
    }

    void set_value(const T& val) {
        if (state_) {
            state_->value = val;
            state_->ready = true;
        }
    }

    void reset() {
        if (state_) {
            state_->ready = false;
        }
    }

private:
    std::shared_ptr<SharedState> state_;
};


#endif /* INC_COMPONENT_EMBEDDEDPROMISE_HPP_ */

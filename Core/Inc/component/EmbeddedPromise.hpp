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
        bool sucess = false;
        T value{};
    };

    Future() = default;
    explicit Future(std::shared_ptr<SharedState> state) : state_(state) {}

    bool is_ready() const {
        return state_ && state_->ready;
    }

    bool is_success() const {
        return is_ready() && (state_->success);
    }

    bool is_failed() const {
        return is_ready() && (!state_->success);
    }

    //bool以外の処理
    //値を破壊せずに参照する（準備ができていればポインタ、未完了なら nullptr）
    const T* peek() const {
    	if (is_ready()) {
    		return &(state_->value);
    	}
    	return nullptr;
    }

    //準備ができていればその値を返し、未完了ならデフォルト値を返す
    T value_or(const T& default_val) const {
    	if (is_ready()) {
    		return state_->value;
    	}
    	return default_val;
    }

    //条件（ラムダ式など）を指定して成功判定を行う
    template <typename Predicate>
    bool is_success_where(Predicate pred) const {
        return is_ready() && pred(state_->value);
    }

    T get() {
        if (!state_) return T{};
        state_->ready = false;
        state_->success = false;
        return state_->value;
    }

    bool valid() const {
        return state_ != nullptr;
    }

    //中身を消し、validを戻す
    //後でpromise.resetが必須
    void clear() {
    	state_.reset();
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

    void set_sucess(const T& val = T{}) {
        if (state_) {
            state_->value = val;
            state_->sucess = true;
            state_->ready = true;
        }
    }

    void set_failed(const T& val = T{}) {
    	if (state_) {
    		state_->value = val;
            state_->success = false;
            state_->ready = true;
        }
    }

    void reset() {
        if (state_) {
            state_->ready = false;
            state_->sucess = false;
        }
    }

private:
    std::shared_ptr<SharedState> state_;
};


#endif /* INC_COMPONENT_EMBEDDEDPROMISE_HPP_ */

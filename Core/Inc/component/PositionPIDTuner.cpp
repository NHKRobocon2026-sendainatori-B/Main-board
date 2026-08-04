/*
 * PositionPIDTuner.cpp
 *
 *  Created on: Aug 4, 2026
 *      Author: nika-
 */

#include <PositionPIDTuner.h>

/* PositionPIDController用チューナー */
PositionPIDTuner::PositionPIDTuner(PositionPIDController* target_pid_)
: target_pid_(target_pid_)
{
	// TODO Auto-generated constructor stub

}

PositionPIDTuner::~PositionPIDTuner() {
	// TODO Auto-generated destructor stub
}

/* 一回転のパルスを設定 */
void PositionPIDTuner::setPulse(float _pulse){
	pulse = _pulse;
}

/* チューニング開始,
 *
 * setting_time : 回す時間
 * target_speed : 回す速さ
 *
 * */
void PositionPIDTuner::start(float settling_time, int16_t target_speed){
	if (target_pid_ == nullptr || target_pid_->speed_pid_ == nullptr) return;

	settling_time_ = settling_time;
	timer_ms_ = 0;
	last_angle = target_pid_->speed_pid_->enc->getAngle();
	filtered_speed = 0.0f;
	t63_speed = static_cast<float>(target_speed * 0.63);

	target_pid_->disable();
	target_pid_->speed_pid_->reset();
	target_pid_->speed_pid_->setTarget(target_speed);

	_state = PosTuningState::MEASURING_SPEED_RESPONSE;
}

/* 更新, 1msごとに呼び出して */
void PositionPIDTuner::update1ms(){
	if (_state == PosTuningState::IDLE || _state == PosTuningState::FINISHED) return;

	timer_ms_++;

	if (_state == PosTuningState::MEASURING_SPEED_RESPONSE){
		int32_t current_angle = target_pid_->speed_pid_->enc->getAngle();
		int32_t diff_angle = current_angle - last_angle;
		last_angle = current_angle;

		float current_rpm = (static_cast<float>(diff_angle) * 60000.0f) / pulse;

		filtered_speed = (1.0f - alpha) * filtered_speed + alpha * current_rpm;

		uint32_t timeout_ms = static_cast<uint32_t>(settling_time_ * 1000.0f);
		if (timeout_ms < 500) timeout_ms = 500;

		if (filtered_speed >= t63_speed || timer_ms_ >= timeout_ms) {
			measured_Tv_ = (float)timer_ms_ / 1000.0f;
		    if (measured_Tv_ < 0.002f) measured_Tv_ = 0.002f;

		    target_pid_->speed_pid_->setTarget(0);
		    _state = PosTuningState::CALCULATING;
		}
	}

	if (_state == PosTuningState::CALCULATING){
		float omega_n = 4.0f / settling_time_;

		float kp = (omega_n * omega_n * measured_Tv_) / pulse * 60.0f;
		float kd = (2.0f * omega_n * measured_Tv_ - 1.0f) / pulse * 60.0f;
		if (kd < 0.0f) kd = 0.0f;

		float ki = (omega_n / 5.0f) * kp;

		target_pid_->setPID(kp, ki, kd);

		target_pid_->enable();
		target_pid_->reset();

		_state = PosTuningState::FINISHED;
	}
}

/* 成功したか主っとく */
bool PositionPIDTuner::isFinished() const {
	return _state == PosTuningState::FINISHED;
}

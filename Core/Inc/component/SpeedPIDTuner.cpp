/*
 * PIDTuner.cpp
 *
 *  Created on: Jul 31, 2026
 *      Author: nika-
 */

#include <SpeedPIDTuner.h>

SpeedPIDTuner::SpeedPIDTuner(SpeedPIDController* _target, TuningMethod _method)
: target(_target), method(_method)
{
	// TODO Auto-generated constructor stub

}

SpeedPIDTuner::~SpeedPIDTuner() {
	// TODO Auto-generated destructor stub
}

void SpeedPIDTuner::start(float _test_pwm_){
	test_pwm_ = _test_pwm_;
	experiment_time_sec_ = 0.0f;
	filtered_speed = 0.0f;
	logger_.start();
	target->disable();
	last_angle = target->enc->getAngle();
	target->act->move((int16_t)test_pwm_);
	state = State::RunningExperiment;
}

void SpeedPIDTuner::stop(){
	logger_.stop();
	target->enable();
	target->act->move(0);
}

void SpeedPIDTuner::setMethod(TuningMethod _method){
	method = _method;
}

void SpeedPIDTuner::setPulse(float _pulse){
	pulse = _pulse;
}

void SpeedPIDTuner::tuneCHR(const StepResponseData& data){
	if (data.input <= 0.0f || data.dead_time_sec <= 0.0f) return;

	float K = data.max_speed / data.input;
	float L = data.dead_time_sec;
	float T = data.t63_sec - data.dead_time_sec;

	if (K <= 0.0f || T <= 0.0f) return;

	float kp = 0.0f, ki = 0.0f, kd = 0.0f;

	if (method == TuningMethod::CHR_0Percent) {
	    kp = (0.60f * T) / (K * L);
	    float Ti = T;
	    float Td = 0.50f * L;
	    ki = kp / Ti;
	    kd = kp * Td;
	} else if (method == TuningMethod::CHR_20Percent) {
	    kp = (0.95f * T) / (K * L);
	    float Ti = 1.35f * T;
	    float Td = 0.47f * L;
	    ki = kp / Ti;
	    kd = kp * Td;
	}

	target->setPID(kp, ki, kd);
	stop();
}

void SpeedPIDTuner::update1ms(){
	if (state != State::RunningExperiment) return;

	experiment_time_sec_ += 0.001f; // 1ms加算

	int32_t current_angle = target->enc->getAngle();
	int32_t diff_angle = current_angle - last_angle;
	last_angle = current_angle;

	float current_rpm = (static_cast<float>(diff_angle) * 60000.0f) / pulse;

	filtered_speed = (1.0f - alpha) * filtered_speed + alpha * current_rpm;

	// 1. データを内部バッファへ自動記録
	logger_.record(experiment_time_sec_, 0.0f, filtered_speed, test_pwm_);

	// 2. 1秒間（1000サンプル）データを取り切ったら自動解析へ
	if (logger_.isFinished()) {
		state = State::AnalyzingAndTuning;

		// 解析とゲイン更新の実行
		processTuning();
	}
}

void SpeedPIDTuner::processTuning(){
	const auto* buffer = logger_.getBuffer();
	uint16_t count = logger_.getSampleCount();

	if (count < 100) {
		state = State::Failed;
	    return;
    }

	float sum_speed = 0.0f;
	uint16_t tail_count = count / 10;
	for (uint16_t i = count - tail_count; i < count; i++) {
		sum_speed += buffer[i].current;
	}
	float max_speed = sum_speed / static_cast<float>(tail_count);

	if (max_speed <= 10.0f) { // モーターが回っていない場合
		state = State::Failed;
	    return;
	}

	float threshold_start = max_speed * 0.05f;  // 動き出し判定(5%)
	float threshold_63    = max_speed * 0.632f; // 63.2%到達判定
	float dead_time_sec = 0.0f;
	float t63_sec = 0.0f;
	bool found_l = false;
	bool found_t63 = false;

	for (uint16_t i = 0; i < count; i++) {
		if (!found_l && buffer[i].current >= threshold_start) {
			dead_time_sec = buffer[i].time_sec;
	        found_l = true;
	    }
	    if (!found_t63 && buffer[i].current >= threshold_63) {
	    	t63_sec = buffer[i].time_sec;
	        found_t63 = true;
	    }
	}

	if (!found_l || !found_t63) {
	    state = State::Failed;
	    return;
	}

	StepResponseData data = {
			.input = test_pwm_,
			.max_speed = max_speed,
			.dead_time_sec = dead_time_sec,
			.t63_sec = t63_sec
	};

	tuneCHR(data);
	state = State::Complete;
}

/* ノイズ抑制の重み追加(0.0 ~ 1.0, 1.0でデータそのまま) */
void SpeedPIDTuner::setAlpha(float _alpha){
	alpha = _alpha;
}

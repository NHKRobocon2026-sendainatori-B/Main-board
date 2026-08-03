/*
 * PIDTuner.h
 *
 *  Created on: Jul 31, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_SPEEDPIDTUNER_H_
#define INC_COMPONENT_SPEEDPIDTUNER_H_

#include <private/StepLogger.hpp>
#include <SpeedPIDController.h>

enum class TuningMethod {
    CHR_0Percent,
    CHR_20Percent,
};

// CHR法（ステップ応答）用データ構造体
struct StepResponseData{
	float input;      // 投入したPWM (%) [例: 30.0]
	float max_speed;      // 到達最高速度 (rpm等) [例: 1500.0]
	float dead_time_sec;  // 無駄時間 L (秒) [例: 0.01]
	float t63_sec;        // 63.2%速度到達時間 (秒) [例: 0.16]
};

class SpeedPIDTuner {
public:
	enum class State {
		Idle,
	    RunningExperiment,
	    AnalyzingAndTuning,
	    Complete,
	    Failed
	};

	SpeedPIDTuner(SpeedPIDController* _target, TuningMethod _method);
	virtual ~SpeedPIDTuner();

	void setMethod(TuningMethod _method);

	void start(float _test_pwm_);
	void stop();

	void update1ms();
	void setAlpha(float _alpha);

private:
	SpeedPIDController* target;
	TuningMethod method;
	State state;

	int32_t last_angle = 0; //速度計測用

	StepLogger<1000> logger_;

	float test_pwm_ = 0.0f;
	float filtered_speed = 0.0f;
	float experiment_time_sec_ = 0.0f;
	float alpha = 0.2f;

	void tuneCHR(const StepResponseData& data);
	void processTuning();
};

#endif /* INC_COMPONENT_SPEEDPIDTUNER_H_ */

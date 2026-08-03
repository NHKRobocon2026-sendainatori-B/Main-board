/*
 * stepLogger.hpp
 *
 *  Created on: Jul 31, 2026
 *      Author: nika-
 */

#pragma once
#include <cstdint>

// 1000サンプル分（1ms周期なら1秒分、2ms周期なら2秒分）
template <uint16_t BUFFER_SIZE = 1000>
class StepLogger {
public:
    struct LogEntry {
        float time_sec;    // 時間 [s]
        float target;      // 目標値 (目標速度や目標位置)
        float current;     // 現在値 (現在速度や現在位置)
        float output;      // 出力値 (PWMや電流コマンド)
    };

    StepLogger() : sample_count_(0), is_logging_(false) {}

    // 計測開始（ステップ応答実験の直前に呼ぶ）
    void start() {
        sample_count_ = 0;
        is_logging_ = true;
    }

    // 計測停止
    void stop() {
        is_logging_ = false;
    }

    // 1ms割り込みなどの制御周期内で毎回呼ぶ
    void record(float time_sec, float target, float current, float output) {
        if (!is_logging_) return;

        if (sample_count_ < BUFFER_SIZE) {
            buffer_[sample_count_] = {time_sec, target, current, output};
            sample_count_++;
        } else {
            is_logging_ = false; // バッファが一杯になったら自動停止
        }
    }

    // 外部からデバッグ時に確認するためのアクセサ
    const LogEntry* getBuffer() const { return buffer_; }
    uint16_t getSampleCount() const { return sample_count_; }
    bool isFinished() const { return !is_logging_ && (sample_count_ >= BUFFER_SIZE); }

private:
    LogEntry buffer_[BUFFER_SIZE]; // 💡 デバッグモードでこの配列の中身を直接覗く！
    uint16_t sample_count_;
    bool is_logging_;
};



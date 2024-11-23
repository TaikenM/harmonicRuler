#include <Arduino.h>
#include "motor_control_utils.h"

// モーター関連のピンの設定
const int motorPWMPin = 6;  // PWM用ピン

double setpoint = 60.0;

void setup() {
    // シリアルモニターの初期化
    Serial.begin(9600);

    // モーターのPWMピンの初期化
    pinMode(motorPWMPin, OUTPUT);

    // エンコーダの割り込み設定
    setupInterrupts(encoderPinA, encoderPinB);
}

void loop() {
    // 制御の更新が必要か判断
    if (shouldUpdateControl()) {
        // 速度の更新
        updateSpeed();

        // PD制御の計算
        calculatePDControl();

        // モーター出力の設定
        setMotorOutput(motorPWMPin);

        // デバッグ情報の表示
        printDebugInfo();
    }
}


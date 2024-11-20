#ifndef MOTOR_CONTROL_UTILS_H
#define MOTOR_CONTROL_UTILS_H

#include <Arduino.h>

// PD制御に関する変数
extern double setpoint;
double input = 0;
double output = 0;
double Kp = 2.0;  // Pゲイン
double Kd = 1.0;  // Dゲイン
double previousError = 0;
unsigned long lastTime = 0;
volatile long encoderCount = 0;
unsigned long lastSampleTime = 0;

void updateEncoder();

// 割り込みの設定
void setupInterrupts(int encoderPinA, int encoderPinB) {
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
}

// 制御の更新が必要か判断
bool shouldUpdateControl() {
    unsigned long currentTime = millis();
    if (currentTime - lastSampleTime >= 100) {
        lastSampleTime = currentTime;
        return true;
    }
    return false;
}

// 速度の更新
void updateSpeed() {
    noInterrupts();  // 割り込みを禁止
    long count = encoderCount;
    encoderCount = 0;  // カウントをリセット
    interrupts();  // 割り込みを許可

    // 速度 (RPM) の計算
    const double pulsesPerRevolution = 24;  // エンコーダのPPRに応じて設定
    input = (count / pulsesPerRevolution) * (60000.0 / 100);  // RPMを計算
}

// PD制御の計算
void calculatePDControl() {
    unsigned long currentTime = millis();
    double error = setpoint - input;
    unsigned long timeDifference = currentTime - lastTime;
    if (timeDifference > 0) {
        double dError = (error - previousError) / timeDifference;  // 誤差の微分を計算
        output = Kp * error + Kd * dError;
    } else {
        output = Kp * error;  // D成分を無視する
    }

    // 出力の範囲を制限
    output = constrain(output, 0, 255);

    // 変数の更新
    previousError = error;
    lastTime = currentTime;
}

// モーター出力の設定
void setMotorOutput(int motorPWM) {
    analogWrite(motorPWM, output);
}

// エンコーダのカウント更新
void updateEncoder() {
    int stateA = digitalRead(2);
    int stateB = digitalRead(3);

    if (stateA == stateB) {
        encoderCount++;
    } else {
        encoderCount--;
    }
}

// デバッグ情報の表示
void printDebugInfo() {
    Serial.print("速度: ");
    Serial.print(input);
    Serial.print(" 出力: ");
    Serial.println(output);
}

#endif // MOTOR_CONTROL_UTILS_H

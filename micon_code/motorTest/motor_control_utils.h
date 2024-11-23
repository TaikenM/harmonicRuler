#ifndef MOTOR_CONTROL_UTILS_H
#define MOTOR_CONTROL_UTILS_H

#include <Arduino.h>

// PD制御に関する変数
extern double setpoint;
double input = 0;
double output = 0;
double Kp = 2.0;  // Pゲイン
double Ki = 0.01;  // Iゲイン
double Kd = 2.0;  // Dゲイン
double previousError = 0;
double integral = 0;
unsigned long lastTime = 0;
volatile long encoderCount = 0;
unsigned long lastSampleTime = 0;

// エンコーダのピンの設定
const int encoderPinA = 2;
const int encoderPinB = 3;
bool aLastState;


void updateEncoder();

// 割り込みの設定
void setupInterrupts(int encoderPinA, int encoderPinB) {
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);

    aLastState = digitalRead(encoderPinA);
}

// 制御の更新が必要か判断
bool shouldUpdateControl() {
    unsigned long currentTime = millis();
    if (currentTime - lastSampleTime >= 50) {
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
    const double pulsesPerRevolution = 48;  // エンコーダのPPRに応じて設定
    input = (count / pulsesPerRevolution) * 60.0 / 0.05;  // 0.1秒ごとの速度に変換
}

// PID制御の計算
void calculatePDControl() {
    unsigned long currentTime = millis();
    double error = setpoint - input;
    //Serial.println(error);
    unsigned long timeDifference = currentTime - lastTime;
    if (timeDifference > 0) {
        integral += error * timeDifference;  // 誤差の積分を計算
        double dError = (error - previousError) / timeDifference;  // 誤差の微分を計算
        output = Kp * error + Ki * integral + Kd * dError;  // PID制御
    } else {
        output = Kp * error + Ki * integral;  // I制御
    }

    // 出力の範囲を制限
    output = constrain(output, 0, 255);

    // 変数の更新
    previousError = error;
    lastTime = currentTime;
}

// モーター出力の設定
void setMotorOutput(int motorPWM) {
    //Serial.println("output: " + String(output));
    analogWrite(motorPWM, output);
}

// エンコーダのカウント更新
void updateEncoder() {
    bool aState = digitalRead(encoderPinA);
    bool bState = digitalRead(encoderPinB);

    if (aState != aLastState) {
        if (aState == HIGH){
            if (bState == LOW) {
                encoderCount++;
            } else {
                encoderCount--;
            }
        } else {
            if (bState == HIGH) {
                encoderCount++;
            } else {
                encoderCount--;
            }
        }
    }
    aLastState = aState;
}

// デバッグ情報の表示
void printDebugInfo() {
    Serial.print("速度: ");
    Serial.print(input);
    Serial.print(" 出力: ");
    Serial.println(output);
}

#endif // MOTOR_CONTROL_UTILS_H

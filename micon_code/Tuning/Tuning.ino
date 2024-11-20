
#include <Arduino.h>
#include <Stepper.h>
#include "motor_control_utils.h"

const int stepsPerRevolution = 200;  // 1回転あたりのステップ数
Stepper stepper(stepsPerRevolution, 7, 9, 8, 10);  // ステッピングモーターオブジェクトの作成

// エンコーダーのピン
const int encoderPinA = 2;
const int encoderPinB = 3;

// モーターのPWMピン
const int motorPWM = 6;

// ソレノイドのピン
const int solenoidPin = 12;

// ステッピングモーターに関する定数
double stepperSpeed = 30;  // ステッピングモーターの速度 (RPM)
int stepDif = 5;  // ステップの差分

// モーターの目標速度
double setpoint = 100;  // 目標速度 (RPM)

// ソレノイドに関する定数
int solenoidOnTime = 100;  // ソレノイドのオン時間 (ミリ秒)

// ピンの初期化
void initializePins() {
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);
    pinMode(motorPWM, OUTPUT);
    pinMode(solenoidPin, OUTPUT);
}

// ステッピングモーターの初期設定
void stepperSetup() {
    stepper.setSpeed(stepperSpeed);
}

// 調律動作
void performTuning() {
    for (int pos = 0; pos <= stepsPerRevolution; pos += stepDif) {
        stepper.step(stepDif);
        delay(500);

        Serial.print("ステップ数: ");
        Serial.println(pos);

        if (pos >= stepsPerRevolution) {
            activateSolenoid();
            break;  // ループを終了
        }
    }
}

// ソレノイドの作動
void activateSolenoid() {
    digitalWrite(solenoidPin, LOW);
    delay(solenoidOnTime);
    digitalWrite(solenoidPin, HIGH);
}

// モーターの位置保持
void holdPosition() {
    while (true) {
        delay(1000);  // 1秒待機
    }
}

void setup() {
    initializePins(); // ピンの初期化
    setupInterrupts(encoderPinA, encoderPinB); // エンコーダの初期化
    Serial.begin(9600);
    stepperSetup(); // ステッピングモーターの初期設定
}

void loop() {
    // PD制御の実行
    if (shouldUpdateControl()) {
        updateSpeed();
        calculatePDControl();
        setMotorOutput(motorPWM);
        printDebugInfo();
    }

    // ステッピングモーターの調律
    performTuning();
    holdPosition();
}

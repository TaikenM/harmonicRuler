#include <Arduino.h>
#include "motor_control_utils.h"
#include "kirakiraboshi.h"
#include <Stepper.h>

const int stepsPerRevolution = 200;  // 1回転あたりのステップ数
Stepper stepper(stepsPerRevolution, 7, 9, 8, 10);  // ステッピングモーターオブジェクトの作成

// ソレノイドのピン
const int solenoidPin = 12;

// モーターのPWMピン
const int motorPWM = 6;

// ステッピングモーターに関する定数
double stepperSpeed = 10;  // ステッピングモーターの速度 (RPM)

// ソレノイドに関する定数
int solenoidOnTime = 100;  // ソレノイドのオン時間 (ミリ秒)

// モーターの速度制御に関する変数
double setpoint = tempo;  // 目標速度 (RPM)

// ピンの初期化
void initializePins() {
    pinMode(solenoidPin, OUTPUT);
    pinMode(motorPWM, OUTPUT);
}

// ステッピングモーターの初期設定
void stepperSetup() {
    stepper.setSpeed(stepperSpeed);
}

// ソレノイドの作動
void activateSolenoid() {
    digitalWrite(solenoidPin, LOW);
    delay(solenoidOnTime);
    digitalWrite(solenoidPin, HIGH);
}

void setup() {
    initializePins(); // ピンの初期化
    Serial.begin(9600);
    stepperSetup(); // ステッピングモーターの初期設定
    setpoint = tempo;  // モーターの回転速度を設定 (RPM)
}

void loop() {
    // モーターの速度制御
    if (shouldUpdateControl()) {
        updateSpeed();
        calculatePDControl();
        setMotorOutput(motorPWM);
    }

    // きらきら星の演奏
    for (int i = 0; i < kirakiraboshiDataSize; i++) {
        
        int steps = kirakiraboshiData[i].steps;
        int duration = kirakiraboshiData[i].duration * 15000 / tempo ;  // テンポに合わせて音符の持続時間を計算

        if (i == 0) {
            // 最初の音符の場合はステッピングモーターを初期位置に戻す
            stepper.step(steps);
        }

        // ソレノイドをオンにする（音を鳴らす）
        activateSolenoid();

        // 次の音程にステッピングモーターを合わせる
        if (i < kirakiraboshiDataSize - 1) {
            int nextSteps = kirakiraboshiData[i + 1].steps;
            stepper.step(nextSteps - steps);  // 次の音程に合わせるためのステップ
            Serial.println(nextSteps);
        }

        // 音の持続時間待機
        delay(duration - solenoidOnTime);
    }

    // 曲が終わったら少し待機
    delay(1000);
}

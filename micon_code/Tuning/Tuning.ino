
#include <Arduino.h>
#include <AccelStepper.h>
#include "motor_control_utils.h"

const int stepsPerRevolution = 200;  // 1回転あたりのステップ数
AccelStepper stepper(AccelStepper::FULL4WIRE, 10, 8, 9, 7);  // ステッピングモーターオブジェクトの作成

const long TuningSteps = 240;  // 調律動作のステップ数


// モーターのPWMピン
const int motorPWM = 6;

// ソレノイドのピン
const int solenoidPin = 12;

// ステッピングモーターに関する定数
const double stepperRPM = 30;  // ステッピングモーターの速度 (RPM)
const double stepperSpeed = stepperRPM * stepsPerRevolution / 60;  // ステッピングモーターの速度 (ステップ/秒)
const double stepperAccel = 100;  // ステッピングモーターの加速度 (ステップ/秒^2)
const long stepDif = 5;  // ステップの差分
const unsigned long stepWaitTime = 1000;  // ステッピングモーターが動いていない時間の閾値
unsigned long previousMillis = 0;  // 前回の時間
long targetStep = stepDif;
bool sol = true;

// モーターの目標速度
double setpoint = 100;  // 目標速度 (RPM)

// ソレノイドに関する定数
int solenoidOnTime = 300;  // ソレノイドのオン時間 (ミリ秒)
unsigned long soleStartTime = 0;  // ソレノイドの作動開始時間

// ピンの初期化
void initializePins() {
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);
    pinMode(motorPWM, OUTPUT);
    pinMode(solenoidPin, OUTPUT);
}

// ステッピングモーターの初期設定
void stepperSetup() {
    stepper.setMaxSpeed(stepperSpeed);
    stepper.setAcceleration(stepperAccel);
    stepper.moveTo(targetStep);
}

// 調律動作
void performTuning() {
    unsigned long currentMillis = millis();
    Serial.println(targetStep);
    stepper.run();
    
    if (stepper.distanceToGo() == 0) {
        if (currentMillis - previousMillis >= stepWaitTime) {
            previousMillis = currentMillis;
            sol = true;
            if (targetStep < TuningSteps) {
                targetStep += stepDif;
                stepper.moveTo(targetStep);
            }
        }
    }
    else if (sol == true){
        activateSolenoid();
        sol = false;
    }
}

// ソレノイドの作動
void activateSolenoid() {
    Serial.println("solenoid called");
    digitalWrite(solenoidPin, LOW);  // ソレノイドをオン
    soleStartTime = millis();  // ソレノイドの作動開始時間を更新
}

void updateSolenoid() {
    if (millis() - soleStartTime >= solenoidOnTime) {
        digitalWrite(solenoidPin, HIGH);  // ソレノイドをオフ
        Serial.println("on");
    }
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
    soleStartTime = millis();  // ソレノイドの作動開始時間の初期化
}

void loop() {
    // PID制御の実行
    if (shouldUpdateControl()) {
        updateSpeed();
        calculatePDControl();
        setMotorOutput(motorPWM);
        printDebugInfo();
    }
    updateSolenoid();
    // ステッピングモーターの調律
    performTuning();
    //holdPosition();
}

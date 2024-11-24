#include <Arduino.h>
#include "motor_control_utils.h"
#include "kirakiraboshi.h"
#include <AccelStepper.h>

const int stepsPerRevolution = 200;  // 1回転あたりのステップ数
Stepper stepper(AccelStepper::FULL4WIRE, 10, 8, 9, 7);  // ステッピングモーターオブジェクトの作成

// ソレノイドのピン
const int solenoidPin = 12;

// モーターのPWMピン
const int motorPWM = 6;

// ステッピングモーターに関する定数
const double stepperRPM = 30;  // ステッピングモーターの速度 (RPM)
const double stepperSpeed = stepperRPM * stepsPerRevolution / 60;  // ステッピングモーターの速度 (ステップ/秒)
const double stepperAccel = 100;  // ステッピングモーターの加速度 (ステップ/秒^2)
int currentNoteNum = 0;
int noteStartTime = 0;

// ソレノイドに関する定数
int solenoidOnTime = 100;  // ソレノイドのオン時間 (ミリ秒)
unsigned long soleStartTime = 0;  // ソレノイドの作動開始時間
bool sol = false;

// モーターの速度制御に関する変数
double setpoint = tempo;  // 目標速度 (RPM)

// ピンの初期化
void initializePins() {
    pinMode(solenoidPin, OUTPUT);
    pinMode(motorPWM, OUTPUT);
}

// ステッピングモーターの初期設定
void stepperSetup() {
    stepper.setMaxSpeed(stepperSpeed);
    stepper.setAcceleration(stepperAccel);
}

// ソレノイドの作動
void activateSolenoid() {
    Serial.println("Solenoid ON");
    digitalWrite(solenoidPin, LOW);
    soleStartTime = millis();
    sol = true;
}

void updateSolenoid() {
    if (millis() - soleStartTime >= solenoidOnTime) {
        Serial.println("Solenoid OFF");
        digitalWrite(solenoidPin, HIGH);
        sol = false;
        stepper.moveTo(kirakiraboshiData[currentNoteNum].steps);
    }
}

void setup() {
    initializePins(); // ピンの初期化
    Serial.begin(9600);
    stepperSetup(); // ステッピングモーターの初期設定
    setpoint = tempo;  // モーターの回転速度を設定 (RPM)
}

void playSong(){
    unsigned long currentMillis = millis();
    stepper.run();
    updateSolenoid();
    
    if (stepper.distanceToGo() == 0){
        if (currentMillis - noteStartTime >= kirakiraboshiData[currentNoteNum].duration){
            noteStartTime = currentMillis;
            activateSolenoid();
            currentNoteNum++;
            if (currentNoteNum == sizeof(kirakiraboshiData)/sizeof(kirakiraboshiData[0])){
                currentNoteNum = 0;
            }

        }
    }
}

void loop() {
    // モーターの速度制御
    if (shouldUpdateControl()) {
        updateSpeed();
        calculatePDControl();
        setMotorOutput(motorPWM);
    }

    playSong();
}

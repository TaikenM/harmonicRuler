#include <Arduino.h>
#include "motor_control_utils.h"
#include "kirakiraboshi.h"
#include <AccelStepper.h>

const int stepsPerRevolution = 200;  // 1回転あたりのステップ数
AccelStepper stepper(AccelStepper::FULL4WIRE, 10, 8, 9, 7);  // ステッピングモーターオブジェクトの作成

// ソレノイドのピン
const int solenoidPin = 12;

// モーターのPWMピン
const int motorPWM = 6;

// ステッピングモーターに関する定数
const double stepperRPM = 30;  // ステッピングモーターの速度 (RPM)
const double stepperSpeed = stepperRPM * stepsPerRevolution / 60;  // ステッピングモーターの速度 (ステップ/秒)
const double stepperAccel = 100;  // ステッピングモーターの加速度 (ステップ/秒^2)
int currentNoteNum = 0;
unsigned long noteStartTime = 0;  // 修正: noteStartTimeのデータ型をunsigned longに変更

// ソレノイドに関する定数
int solenoidOnTime = 300;  // ソレノイドのオン時間 (ミリ秒)
unsigned long soleStartTime = 0;  // ソレノイドの作動開始時間
bool sol = false;

// モーターの速度制御に関する変数
double setpoint = tempo;  // 目標速度 (RPM)

// ピンの初期化
void initializePins() {
    pinMode(solenoidPin, OUTPUT);
    pinMode(motorPWM, OUTPUT);
    digitalWrite(solenoidPin, HIGH);  // 初期状態をOFFにしておく
}

// ステッピングモーターの初期設定
void stepperSetup() {
    stepper.setMaxSpeed(stepperSpeed);
    stepper.setAcceleration(stepperAccel);
    stepper.moveTo(kirakiraboshiData[currentNoteNum].steps);
    noteStartTime = millis();
}

// ソレノイドの作動
void activateSolenoid() {
    Serial.println("Solenoid ON");
    digitalWrite(solenoidPin, LOW);
    soleStartTime = millis();
    sol = true;
}

void updateSolenoid() {
    if (sol && (millis() - soleStartTime >= solenoidOnTime)) {
        Serial.println("Solenoid OFF");
        digitalWrite(solenoidPin, HIGH);
        sol = false;
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
    stepper.run();  // ステッピングモーターを動作させる
    updateSolenoid();  // ソレノイドの状態を更新

    // 音符の持続時間を確認
    if (currentMillis - noteStartTime >= kirakiraboshiData[currentNoteNum].duration * 1000 * 60 / (tempo * 4)) {
        if (stepper.distanceToGo() == 0) {
            noteStartTime = currentMillis;
            activateSolenoid();  // 次の音符に合わせてソレノイドをアクチュエート
            currentNoteNum++;
            
            // 曲の終わりに達した場合、最初に戻る
            if (currentNoteNum == sizeof(kirakiraboshiData) / sizeof(kirakiraboshiData[0])) {
                currentNoteNum = 0;
            }
            
            // ステッピングモーターの次の目標位置を設定
            stepper.moveTo(kirakiraboshiData[currentNoteNum].steps);
            Serial.print("Moving to note: ");
            Serial.println(currentNoteNum);
        } else {
            Serial.println("Stepper still moving to target.");
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

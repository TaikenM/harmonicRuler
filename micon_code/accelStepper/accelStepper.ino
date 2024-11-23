#include <AccelStepper.h>

// 4線式ステッピングモーターのピン設定 (ピン8, 9, 10, 11)
AccelStepper stepper(AccelStepper::FULL4WIRE, 7, 9, 8, 10);

void setup() {
  // シリアルモニターを使用するための初期化
  Serial.begin(9600);

  // 最大速度と加速度の設定
  stepper.setMaxSpeed(400);   // 最大速度 (ステップ/秒)
  stepper.setAcceleration(200); // 加速度 (ステップ/秒^2)

  // 初期位置への移動 (例えば1000ステップ先)
  stepper.moveTo(200);
}

void loop() {
  // モーターを動かすためのrun関数を呼び出す
  stepper.run();

  // 現在の位置と目標位置をシリアルモニタに出力する
  /*Serial.print("Current Position: ");
  Serial.print(stepper.currentPosition());
  Serial.print(" / Target Position: ");
  Serial.println(stepper.targetPosition());
*/
  // 目標位置に到達したら反対方向へ目標位置を変える
  if (stepper.distanceToGo() == 0) {
    // 現在の位置が1000であれば、0に戻す
    if (stepper.currentPosition() == 200) {
      stepper.moveTo(0);
    } else {
      stepper.moveTo(200);
    }
  }

  // 適度な遅延を入れてシリアルモニタが読みやすくなるようにする
  //delay(100);
}

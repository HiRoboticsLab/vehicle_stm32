#include <Arduino.h>
#include <ArduinoJson.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <OneButton.h>
#include <PID_v1.h>
#include <Ticker.h>
#include <Wire.h>
#include <stdint.h>

#include "config.h"
#include "wheel.h"

#define PI 3.14159265358979323846

void adjust();
void reportInfo();
void turnLight();
void blink();

String stateTurnLight = "off";

// PID
double InputW1, OutputW1, SetpointW1 = 0;
double InputW2, OutputW2, SetpointW2 = 0;
double InputW3, OutputW3, SetpointW3 = 0;
double InputW4, OutputW4, SetpointW4 = 0;

double Kp = 0.01, Ki = 1.0, Kd = 0.001;
PID PIDW1(&InputW1, &OutputW1, &SetpointW1, Kp, Ki, Kd, DIRECT);
PID PIDW2(&InputW2, &OutputW2, &SetpointW2, Kp, Ki, Kd, DIRECT);
PID PIDW3(&InputW3, &OutputW3, &SetpointW3, Kp, Ki, Kd, DIRECT);
PID PIDW4(&InputW4, &OutputW4, &SetpointW4, Kp, Ki, Kd, DIRECT);

// 定时器
Ticker timerAdjust(adjust, 50);
Ticker timerTurnlight(turnLight, 500);
Ticker timerReport(reportInfo, 50);
Ticker timerBlink(blink, 100);

// 轮子（PCB引脚顺序有调整，所以最后一个均为true）
Wheel wheel1(pin_wheel_1_in1, pin_wheel_1_in2, pin_wheel_1_code1,
             pin_wheel_1_code2, false);
Wheel wheel2(pin_wheel_2_in1, pin_wheel_2_in2, pin_wheel_2_code1,
             pin_wheel_2_code2, false);
Wheel wheel3(pin_wheel_3_in1, pin_wheel_3_in2, pin_wheel_3_code1,
             pin_wheel_3_code2, true);
Wheel wheel4(pin_wheel_4_in1, pin_wheel_4_in2, pin_wheel_4_code1,
             pin_wheel_4_code2, false);

// mpu
// Quaternion q;
// uint8_t fifoBuffer[64];  // FIFO storage buffer
// VectorFloat gravity;     // [x, y, z]            gravity vector
// float ypr[3];            // [yaw, pitch, roll]
// MPU6050 mpu;
// VectorInt16 aa;
// VectorInt16 aaReal;      // [x, y, z]            gravity-free accel sensor
// measurements VectorInt16 aaWorld;

// 按键
OneButton button(pin_btn, true);

// 硬件测试模式
void testMode();
// 向上位机请求重启蓝牙
void resetBLE();
bool FLAG_BLE_RESET = false;

// 轮子外部中断回调函数
void callbackWheel1() { wheel1.count++; }
void callbackWheel2() { wheel2.count++; }
void callbackWheel3() { wheel3.count++; }
void callbackWheel4() { wheel4.count++; }

// void callbackWheel1() { Serial1.print("1"); }
// void callbackWheel2() { Serial1.print("2"); }
// void callbackWheel3() { Serial1.print("3"); }
// void callbackWheel4() { Serial1.print("4"); }

// 轮子开外部中断
void attachAll() {
  wheel1.attach(callbackWheel1);
  wheel2.attach(callbackWheel2);
  wheel3.attach(callbackWheel3);
  wheel4.attach(callbackWheel4);
}

// 轮子关外部中断
void detachAll() {
  wheel1.detach();
  wheel2.detach();
  wheel3.detach();
  wheel4.detach();
}

void reportInfo() {
  DynamicJsonDocument report(512);

  if (FLAG_BLE_RESET) {
    report["ble"] = "restart";
    FLAG_BLE_RESET = false;
  } else {
    // report["light"] = digitalRead(pin_light) ? "off" : "on";
    // report["turnLight"] = stateTurnLight;

    // mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    // mpu.dmpGetQuaternion(&q, fifoBuffer);
    // mpu.dmpGetGravity(&gravity, &q);
    // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    report["ypr"][0] = 0;
    // 对应车辆坐标系
    report["ypr"][1] = 0;
    report["ypr"][2] = 0;

    // mpu.dmpGetAccel(&aa, fifoBuffer);
    // mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    // mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    report["accel"][0] = 0;
    report["accel"][1] = 0;
    report["accel"][2] = 0;

    report["quaternion"][0] = 0;
    report["quaternion"][1] = 0;
    report["quaternion"][2] = 0;
    report["quaternion"][3] = 0;

    //
    report["wheel"][0] = wheel3.getCountByDir();
    report["wheel"][1] = wheel4.getCountByDir();
    report["wheel"][2] = SetpointW4;
    report["wheel"][3] = SetpointW4;
  }

  adjust();

  String str = "";
  serializeJson(report, str);
  Serial3.println(str);
}

void blink() {
  digitalWrite(pin_light_D7, !digitalRead(pin_light_D7));
  digitalWrite(pin_light_D8, !digitalRead(pin_light_D8));
  digitalWrite(pin_light_D9, !digitalRead(pin_light_D9));
  digitalWrite(pin_light_D10, !digitalRead(pin_light_D10));
}

// PID调整
void adjust() {
  // detachAll();
  wheel1.setSpeed(OutputW1);
  wheel2.setSpeed(OutputW2);
  wheel3.setSpeed(OutputW3);
  wheel4.setSpeed(OutputW4);

  if (SetpointW1 - wheel1.count != 3) {
    InputW1 = wheel1.count;
  } else {
    InputW1 = SetpointW1;
  }

  if (SetpointW2 - wheel2.count != 3) {
    InputW2 = wheel2.count;
  } else {
    InputW2 = SetpointW2;
  }

  if (SetpointW3 - wheel3.count != 3) {
    InputW3 = wheel3.count;
  } else {
    InputW3 = SetpointW3;
  }

  if (SetpointW4 - wheel4.count != 3) {
    InputW4 = wheel4.count;
  } else {
    InputW4 = SetpointW4;
  }

  wheel1.count = 0;
  wheel2.count = 0;
  wheel3.count = 0;
  wheel4.count = 0;
  // attachAll();
}

void initLight() {
  // 前面板灯
  pinMode(pin_light, OUTPUT);
  pinMode(pin_light_left, OUTPUT);
  pinMode(pin_light_right, OUTPUT);
  //
  digitalWrite(pin_light, HIGH);
  digitalWrite(pin_light_left, LOW);
  digitalWrite(pin_light_right, LOW);
  // 板载灯
  pinMode(pin_light_D7, OUTPUT);
  pinMode(pin_light_D8, OUTPUT);
  pinMode(pin_light_D9, OUTPUT);
  pinMode(pin_light_D10, OUTPUT);
  //
  digitalWrite(pin_light_D7, HIGH);
  digitalWrite(pin_light_D8, HIGH);
  digitalWrite(pin_light_D9, HIGH);
  digitalWrite(pin_light_D10, HIGH);
}

// 复位转向灯
void resetTurnLight() {
  digitalWrite(pin_light_left, LOW);
  digitalWrite(pin_light_right, LOW);
}

// 转向灯控制
void turnLight() {
  if (!stateTurnLight.equals("off")) {
    if (stateTurnLight.equals("on")) {
      digitalWrite(pin_light_left, !digitalRead(pin_light_left));
      digitalWrite(pin_light_right, !digitalRead(pin_light_right));
    }
    if (stateTurnLight.equals("left")) {
      digitalWrite(pin_light_left, !digitalRead(pin_light_left));
    }
    if (stateTurnLight.equals("right")) {
      digitalWrite(pin_light_right, !digitalRead(pin_light_right));
    }
  }
}

// 大灯控制
void light(bool state) { digitalWrite(pin_light, !state); }
void light() { digitalWrite(pin_light, !digitalRead(pin_light)); }

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200);
  Serial1.println("init done");

  Serial2.begin(115200);
  Serial2.println("init done");

  Serial3.begin(115200);
  Serial3.println("init done");

  // 四个车轮PWM频率，影响声音
  analogWriteFrequency(20000);
  // 电机H桥使能
  pinMode(pin_hr8833_left, OUTPUT);
  pinMode(pin_hr8833_right, OUTPUT);
  digitalWrite(pin_hr8833_left, HIGH);
  digitalWrite(pin_hr8833_right, HIGH);

  // HAL_NVIC_SetPriority(USART3_IRQn, 3, 1);

  // mpu6050
  // Wire.setSDA(pin_mpu_sda);
  // Wire.setSCL(pin_mpu_scl);
  // Wire.begin();

  // Wire.setClock(400000);

  // bool connected = false;

  // while (!connected) {
  //   // mpu.reset();
  //   // delay(1000);
  //   mpu.initialize();
  //   delay(2000);

  //   connected = mpu.testConnection();
  //   connected ? Serial1.println("[ok] imu") : Serial1.println("[error] imu");
  // }

  // mpu.dmpInitialize();

  // mpu.CalibrateAccel(6);
  // mpu.CalibrateGyro(6);
  // mpu.PrintActiveOffsets();
  // // turn on the DMP, now that it's ready
  // Serial1.println("Enabling DMP...");
  // mpu.setDMPEnabled(true);

  // 开启码盘中断
  attachAll();

  // 初始化灯光
  initLight();

  // 开启定时任务
  timerAdjust.start();
  timerReport.start();
  timerTurnlight.start();
  timerBlink.start();

  PIDW1.SetMode(AUTOMATIC);
  PIDW2.SetMode(AUTOMATIC);
  PIDW3.SetMode(AUTOMATIC);
  PIDW4.SetMode(AUTOMATIC);

  // 设备测试按钮
  button.attachDoubleClick(testMode);
  button.attachLongPressStart(resetBLE);
}

String cmd = "";
DynamicJsonDocument doc(256);

void loop() {
  button.tick();

  timerTurnlight.update();
  timerBlink.update();
  timerReport.update();

  PIDW1.Compute();
  PIDW2.Compute();
  PIDW3.Compute();
  PIDW4.Compute();

  if (Serial3.available()) {
    char temp = char(Serial3.read());
    cmd += temp;
    if (temp == '\n') {
      deserializeJson(doc, cmd);

      if (!doc.containsKey("to")) {
        cmd = "";
        return;
      }

      if (doc["to"] == "wheel") {
        double left = doc["data"][0];
        double right = doc["data"][1];

        double w1s = left;
        double w2s = right;
        double w3s = left;
        double w4s = right;

        // if(left * (-1) == right){
        //   if(doc["data"][0] < 0){
        //     w4s = 0;
        //   }
        //   if(doc["data"][1] < 0){
        //     w3s = 0;
        //   }
        // }

        wheel1.checkSpeed(w1s);
        wheel2.checkSpeed(w2s);
        wheel3.checkSpeed(w3s);
        wheel4.checkSpeed(w4s);

        SetpointW1 = abs(w1s);
        SetpointW2 = abs(w2s);
        SetpointW3 = abs(w3s);
        SetpointW4 = abs(w4s);

        // 最大速度限制保护驱动芯片
        // abs(w1s) > 250 ? SetpointW1 = 250 : SetpointW1 = abs(w1s);
        // abs(w2s) > 250 ? SetpointW2 = 250 : SetpointW2 = abs(w2s);
        // abs(w3s) > 250 ? SetpointW3 = 250 : SetpointW3 = abs(w3s);
        // abs(w4s) > 250 ? SetpointW4 = 250 : SetpointW4 = abs(w4s);
      }

      if (doc["to"] == "light") {
        String arg = doc["data"];
        if (arg.equals("head")) {
          light();
        }
        if (arg.equals("left")) {
          resetTurnLight();
          if (!stateTurnLight.equals("left")) {
            stateTurnLight = "left";
          } else {
            stateTurnLight = "off";
          }
        }
        if (arg.equals("right")) {
          resetTurnLight();
          if (!stateTurnLight.equals("right")) {
            stateTurnLight = "right";
          } else {
            stateTurnLight = "off";
          }
        }
        if (arg.equals("both")) {
          resetTurnLight();
          if (!stateTurnLight.equals("on")) {
            stateTurnLight = "on";
          } else {
            stateTurnLight = "off";
          }
        }
      }

      if (doc["to"] == "pid") {
        PIDW1.SetTunings(doc["data"][0], doc["data"][1], doc["data"][2]);
        PIDW2.SetTunings(doc["data"][0], doc["data"][1], doc["data"][2]);
        PIDW3.SetTunings(doc["data"][0], doc["data"][1], doc["data"][2]);
        PIDW4.SetTunings(doc["data"][0], doc["data"][1], doc["data"][2]);
      }

      if (doc["to"] == "esp") {
        // 里面已经有了\n，就不用println了
        Serial2.print(cmd);
      }

      cmd = "";
    }
  }
}

void resetBLE() { FLAG_BLE_RESET = true; }

/**
 * ==============
 *   硬件测试模式
 * ==============
 */
void testLight() {
  // 前面板灯
  digitalWrite(pin_light, !digitalRead(pin_light));
  digitalWrite(pin_light_left, !digitalRead(pin_light_left));
  digitalWrite(pin_light_right, !digitalRead(pin_light_right));
  // 板载灯
  digitalWrite(pin_light_D7, !digitalRead(pin_light_D7));
  digitalWrite(pin_light_D8, !digitalRead(pin_light_D8));
  digitalWrite(pin_light_D9, !digitalRead(pin_light_D9));
  digitalWrite(pin_light_D10, !digitalRead(pin_light_D10));
  // 读取imu，控制车轮
  // mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
  // mpu.dmpGetQuaternion(&q, fifoBuffer);
  // mpu.dmpGetGravity(&gravity, &q);
  // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  // // float是4Byte -> 1b + 8b + 23b
  // // double是8Byte -> 1b + 11b + 52b
  // double temp = ypr[0] * 180 / PI;
  // int lv = 0, rv = 0;
  // if (temp > 30) {
  //   lv = -60;
  //   rv = 60;
  // } else if (temp < -30) {
  //   lv = 60;
  //   rv = -60;
  // } else {
  //   lv = 0;
  //   rv = 0;
  // }
  // // 控制车轮
  // wheel1.checkSpeed(lv);
  // wheel2.checkSpeed(rv);
  // wheel3.checkSpeed(lv);
  // wheel4.checkSpeed(rv);
  // SetpointW1 = abs(lv);
  // SetpointW2 = abs(rv);
  // SetpointW3 = abs(lv);
  // SetpointW4 = abs(rv);
}

int lv = 0, rv = 0;
int mode = 0;
int speed = 50;

void testWheels() {
  if (mode % 2) {
    lv = 0;
    rv = 0;
  }
  switch (mode) {
    case 0:
      lv = speed;
      rv = speed;
      break;
    case 2:
      lv = -speed;
      rv = -speed;
      break;
    case 4:
      lv = -speed;
      rv = speed;
      break;
    case 6:
      lv = speed;
      rv = -speed;
      break;
    default:
      break;
  }
  mode++;
  if (mode > 7) {
    mode = 0;
  }

  Serial1.println(mode % 2);

  wheel1.checkSpeed(lv);
  wheel2.checkSpeed(rv);
  wheel3.checkSpeed(lv);
  wheel4.checkSpeed(rv);
  if (lv != 0 && rv != 0) {
    SetpointW1 = abs(lv);
    SetpointW2 = abs(rv);
    SetpointW3 = abs(lv);
    SetpointW4 = abs(rv);
  }
}

// 测试模式定时器
Ticker timerTestLight(testLight, 1000);
Ticker timerTestWheels(testWheels, 2000);

void testMode() {
  Serial1.println("double click");
  timerTestLight.start();
  timerTestWheels.start();
  while (1) {
    timerTestLight.update();
    timerTestWheels.update();

    timerAdjust.update();
    PIDW1.Compute();
    PIDW2.Compute();
    PIDW3.Compute();
    PIDW4.Compute();
  }
}
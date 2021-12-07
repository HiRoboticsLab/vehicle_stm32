#include "wheel.h"

#include <Arduino.h>

Wheel::Wheel(uint32_t in1, uint32_t in2, uint32_t code1, uint32_t code2,
             bool direction) {
  this->pin_in1 = in1;
  this->pin_in2 = in2;
  this->pin_code1 = code1;
  this->pin_code2 = code2;
  this->dir_default = direction;
  this->dir_now = direction;

  // 解决电机引脚占用IIC被默认初始化问题，防止波形高度不够
  pinMode(code1, INPUT_PULLDOWN);
  pinMode(code2, INPUT_PULLDOWN);

  pinMode(in1, INPUT_PULLDOWN);
  pinMode(in2, INPUT_PULLDOWN);

  disable();
}

void Wheel::setSpeed(int pwm) {
  // 如果是停车状态，忽略pid计算后的调用
  if (is_disable) {
    disable();
    return;
  }
  // // 最大PWM限制，保护驱动芯片
  // if (pwm > 200) {
  //   pwm = 200;
  // }
  if (dir_now) {
    analogWrite(pin_in1, pwm);
    analogWrite(pin_in2, 0);
  } else {
    analogWrite(pin_in1, 0);
    analogWrite(pin_in2, pwm);
  }
}

void Wheel::checkSpeed(int pwm) {
  if (pwm == 0) {
    is_disable = true;
    disable();
  } else {
    is_disable = false;
    pwm < 0 ? dir_now = !dir_default : dir_now = dir_default;
  }
}

void Wheel::disable() {
  analogWrite(this->pin_in1, 0);
  analogWrite(this->pin_in2, 0);
}

void Wheel::attach(void (*callback)(void)) {
  attachInterrupt(pin_code1, callback, RISING);
  attachInterrupt(pin_code2, callback, RISING);
}

void Wheel::detach() { detachInterrupt(pin_code1); }

/**
 * 通过方向获取计数
 */
int16_t Wheel::getCountByDir() {
  if (dir_now == dir_default) {
    return count;
  } else {
    return -count;
  }
}

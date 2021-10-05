#ifndef __WHEEL_H__
#define __WHEEL_H__

#include <stdint.h>

class Wheel {
 private:
  uint32_t pin_in1;
  uint32_t pin_in2;
  uint32_t pin_code1;
  uint32_t pin_code2;
  bool dir_default;
  bool dir_now;
  bool is_disable = true;

  void disable();
  void enable(bool dir);

 public:
  Wheel(uint32_t in1, uint32_t in2, uint32_t code1, uint32_t code2,
        bool direction);
  void setSpeed(int pwm);
  void checkSpeed(int pwm);
  void attach(void (*callback)(void));
  void detach();

  //   int64_t count = 0;
  //   int64_t last_count = 0;
  int16_t count = 0;

  int16_t getCountByDir();
};

#endif

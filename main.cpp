#include "motor.h"
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#define FL_hip_max 6.f
#define FL_hip_min -6.f
#define FL_hip_start_joint 1.42f

int main() {
  // Uart serial("/dev/ttyUSB0");
  std::shared_ptr<Uart>  uart = std::make_shared<Uart>("/dev/ttyUSB0");
  std::shared_ptr<motor> FL_motor_0=std::make_shared<motor>(FL_hip_max,FL_hip_min ,uart);

  while (1) {
    FL_motor_0->set_motor(0,0.015,2,0,-7,-0.3);
  }
  return 0;
}

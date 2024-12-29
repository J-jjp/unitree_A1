#include "motor.h"
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#define FL_calf_dir 1

#define FR_calf_dir -1
#define PI 3.141592653589793f
int main() {
  // Uart serial("/dev/ttyUSB0");
  std::shared_ptr<Uart>  uart = std::make_shared<Uart>("/dev/ttyUSB0");
  float calf_ammount= (PI-0.3628)*9.1;
  float thigh_ammount= (PI-0.3628)*9.1;
  std::shared_ptr<motor> FL_motor_0=std::make_shared<motor>(2,calf_ammount,-1 ,uart);

  while (1) {
    FL_motor_0->set_motor(0,0,0,0,0);
    std::cout<<"zero:"<<FL_motor_0->zero_pose<<std::endl;
    std::cout<<FL_motor_0->recv_pose()<<std::endl;
  }
  return 0;
}

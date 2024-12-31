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
  std::shared_ptr<Uart>  uart_0 = std::make_shared<Uart>("/dev/ttyUSB0");
  std::shared_ptr<Uart>  uart_1 = std::make_shared<Uart>("/dev/ttyUSB1");
  std::shared_ptr<Uart>  uart_2 = std::make_shared<Uart>("/dev/ttyUSB2");
  std::shared_ptr<Uart>  uart_3 = std::make_shared<Uart>("/dev/ttyUSB3");
  float calf_ammount= (PI-0.3628)*9.1;
  float thigh_ammount=-(PI/2-0.3628)*9.1;

  std::shared_ptr<motor> FL_motor_0=std::make_shared<motor>(0,0,1 ,uart_0);
  std::shared_ptr<motor> FR_motor_0=std::make_shared<motor>(0,0,1 ,uart_1);
  std::shared_ptr<motor> RL_motor_0=std::make_shared<motor>(0,0,-1 ,uart_2);
  std::shared_ptr<motor> RR_motor_0=std::make_shared<motor>(0,0,-1 ,uart_3);

  std::shared_ptr<motor> FL_motor_1=std::make_shared<motor>(1,thigh_ammount,1 ,uart_0);
  std::shared_ptr<motor> FR_motor_1=std::make_shared<motor>(1,thigh_ammount,-1 ,uart_1);
  std::shared_ptr<motor> RL_motor_1=std::make_shared<motor>(1,thigh_ammount,1 ,uart_2);
  std::shared_ptr<motor> RR_motor_1=std::make_shared<motor>(1,thigh_ammount,-1 ,uart_3);

  std::shared_ptr<motor> FL_motor_2=std::make_shared<motor>(2,calf_ammount,1 ,uart_0);
  std::shared_ptr<motor> FR_motor_2=std::make_shared<motor>(2,calf_ammount,-1 ,uart_1);
  std::shared_ptr<motor> RL_motor_2=std::make_shared<motor>(2,calf_ammount,1 ,uart_2);
  std::shared_ptr<motor> RR_motor_2=std::make_shared<motor>(2,calf_ammount,-1 ,uart_3);
  float _percent = 0;
  float _duration = 1000;
  float FL_start_2 = FL_motor_2->recv_pose();
  float FR_start_2 = FR_motor_2->recv_pose();
  float RL_start_2 = RL_motor_2->recv_pose();
  float RR_start_2 = RR_motor_2->recv_pose();

  float FL_start_1 = FL_motor_1->recv_pose();
  float FR_start_1 = FR_motor_1->recv_pose();
  float RL_start_1 = RL_motor_1->recv_pose();
  float RR_start_1 = RR_motor_1->recv_pose();

  float FL_start_0 = FL_motor_0->recv_pose();
  float FR_start_0 = FR_motor_0->recv_pose();
  float RL_start_0 = RL_motor_0->recv_pose();
  float RR_start_0 = RR_motor_0->recv_pose();
  while (1) {
    _percent+=(float)1/_duration;
    _percent = _percent > 1 ? 1 : _percent;
    float FL_pose_2 = (1 - _percent)*FL_start_2+ _percent*(-1.5);
    float FR_pose_2 = (1 - _percent)*FR_start_2+ _percent*(-1.5);
    float RL_pose_2 = (1 - _percent)*RL_start_2+ _percent*(-1.5);
    float RR_pose_2 = (1 - _percent)*RR_start_2+ _percent*(-1.5);

    float FL_pose_1 = (1 - _percent)*FL_start_1+ _percent*(0.8);
    float FR_pose_1 = (1 - _percent)*FR_start_1+ _percent*(0.8);
    float RL_pose_1 = (1 - _percent)*RL_start_1+ _percent*(1);
    float RR_pose_1 = (1 - _percent)*RR_start_1+ _percent*(1);

    float FL_pose_0 = (1 - _percent)*FL_start_0+ _percent*(-0.1);
    float FR_pose_0=  (1 - _percent)*FR_start_0+ _percent*(0.1);
    float RL_pose_0 = (1 - _percent)*RL_start_0+ _percent*(-0.1);
    float RR_pose_0 = (1 - _percent)*RR_start_0+ _percent*(0.1);

    // FL_motor_2->set_motor(0.02,2,FL_pose_2,1,0.05);
    // FR_motor_2->set_motor(0.02,2,FR_pose_2,-1,-0.05);
    // RL_motor_2->set_motor(0.02,2,RR_pose_2,1,0.05);
    // RR_motor_2->set_motor(0.02,2,RR_pose_2,-1,-0.05);
          FL_motor_0->set_motor(0.0,0,0,0,0);
          FL_motor_1->set_motor(0.0,0,0,0,0);
    // FL_motor_1->set_motor(0.015,2,FL_pose_1,-4,-0.3);
    std::cout<<"FL1"<<(int)FL_motor_1->data.motor_id<<FL_motor_1->recv_pose();
    FR_motor_1->set_motor(0.015,2,FR_pose_1,4,0.3);
    RL_motor_1->set_motor(0.015,2,RL_pose_1,-4,-0.3);
    RR_motor_1->set_motor(0.015,2,RR_pose_1,4,0.3);
    // RR_motor_1->set_motor(0.015,2,RR_pose_1,4,0.3);
      FL_motor_2->set_motor(0.0,0,0,0,0);
    FR_motor_2->set_motor(0.0,0,0,0,0);
      RL_motor_2->set_motor(0.0,0,0,0,0);
          RR_motor_2->set_motor(0.0,0,0,0,0);
      // FL_motor_0->set_motor(0.015,2,FL_pose_0,-4,-0.3);
      // FR_motor_0->set_motor(0.015,2,FR_pose_0,4,0.3);
      // RL_motor_0->set_motor(0.015,2,RL_pose_0,4,0.3);
      // RR_motor_0->set_motor(0.015,2,RR_pose_0,-4,-0.3);
    std::cout<<"\tFL"<<FL_motor_0->recv_pose()<<"\tFR:"<<FL_motor_1->recv_pose()<<"\tRL"<<FL_motor_2->recv_pose()<<"\tRR"<<
    RR_motor_1->recv_pose()<<std::endl;
    // std::cout<<"FL"<<FR_motor_0->recv_pose()<<"\tPOS:"<<FR_motor_0->data.Pos<<"\tmax"<<FR_motor_0->max_pose<<"\tmin"<<
    // FR_motor_0->min_pose<<std::endl;
    // std::cout<<"FL"<<RL_motor_0->recv_pose()<<"\tPOS:"<<RL_motor_0->data.Pos<<"\tmax"<<RL_motor_0->max_pose<<"\tmin"<<
    // RL_motor_0->min_pose<<std::endl;
    // std::cout<<"FL"<<FR_motor_0->recv_pose()<<"\tPOS:"<<FR_motor_0->data.Pos<<"\tmax"<<FR_motor_0->max_pose<<"\tmin"<<
    // FR_motor_0->min_pose<<std::endl;
    // std::cout<<"FL"<<FR_motor_0->recv_pose()<<"\tPOS:"<<FR_motor_0->data.Pos<<"\tmax"<<FR_motor_0->max_pose<<"\tmin"<<
    // FR_motor_0->min_pose<<std::endl;
    // std::cout<<"zero:FL"<<FL_motor_0->zero_pose<<"\tFR:"<<FR_motor_0->zero_pose
    // <<"\tRL:"<<RL_motor_0->zero_pose<<"\tRR:"<<RR_motor_0->zero_pose<<"pose"<<pose<<std::endl;
    // std::cout<<"recv:FL"<<FL_motor_0->recv_pose()<<"\tFR:"<<FR_motor_0->recv_pose()
    // <<"\tRL:"<<RL_motor_0->recv_pose()<<"\tRR:"<<RR_motor_0->recv_pose()<<std::endl;
  }
  return 0;
}

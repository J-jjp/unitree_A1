#include "leg_control.h"


int main(int argc, char** argv){
    std::vector<int> FL_dir={1,1,1};
    std::vector<int> FR_dir={1,-1,-1};
    std::vector<int> RL_dir={-1,1,1};
    std::vector<int> RR_dir={-1,-1,-1};
    std::shared_ptr<leg_control>  FL_leg = std::make_shared<leg_control>("FL", "/dev/ttyUSB0",FL_dir);
    std::shared_ptr<leg_control>  FR_leg = std::make_shared<leg_control>("FR", "/dev/ttyUSB1",FR_dir);
    std::shared_ptr<leg_control>  RL_leg = std::make_shared<leg_control>("RL", "/dev/ttyUSB2",RL_dir);
    std::shared_ptr<leg_control>  RR_leg = std::make_shared<leg_control>("RR", "/dev/ttyUSB3",RR_dir);
    std::vector<float> FL_kp={0.02,0.02,0.02};
    std::vector<float> kp={0.0,0.0,0.0};
    std::vector<float> kd={0,0,0};
    std::vector<float> q={0,0,0};
    std::vector<float> FL_q={0,0,0};
    std::vector<float> FR_q={0,0,0};
    std::vector<float> RL_q={0,0,0};
    std::vector<float> RR_q={0,0,0};
    std::vector<float> dq={0,0,0};
    std::vector<float> FL_tau={0,0,0.15};
    std::vector<float> FR_tau={0,0,-0.15};
    std::vector<float> RL_tau={0,0,0.35};
    std::vector<float> RR_tau={0,0,-0.35};
    float _percent = 0;
    float _duration = 1000;
    while (1)
    {
      _percent+=(float)1/_duration;
      _percent = _percent > 1 ? 1 : _percent;
      FL_q[0]= (1 - _percent)*FL_leg->leg_start[0]+ _percent*(-0.1);
      FL_q[1]= (1 - _percent)*FL_leg->leg_start[1]+ _percent*(0.8);
      FL_q[2]= (1 - _percent)*FL_leg->leg_start[2]+ _percent*(-1.5);

      FR_q[0]= (1 - _percent)*FR_leg->leg_start[0]+ _percent*(0.1);
      FR_q[1]= (1 - _percent)*FR_leg->leg_start[1]+ _percent*(0.8);
      FR_q[2]= (1 - _percent)*FR_leg->leg_start[2]+ _percent*(-1.5);

      RL_q[0]= (1 - _percent)*RL_leg->leg_start[0]+ _percent*(-0.1);
      RL_q[1]= (1 - _percent)*RL_leg->leg_start[1]+ _percent*(1);
      RL_q[2]= (1 - _percent)*RL_leg->leg_start[2]+ _percent*(-1.5);

      RR_q[0]= (1 - _percent)*RR_leg->leg_start[0]+ _percent*(0.1);
      RR_q[1]= (1 - _percent)*RR_leg->leg_start[1]+ _percent*(1);
      RR_q[2]= (1 - _percent)*RR_leg->leg_start[2]+ _percent*(-1.5);
      FL_leg->set_motor_cmd(FL_kp,kd,FL_q,dq,FL_tau);
      FR_leg->set_motor_cmd(FL_kp,kd,FR_q,dq,FR_tau);
      RL_leg->set_motor_cmd(FL_kp,kd,RL_q,dq,RL_tau);
      RR_leg->set_motor_cmd(FL_kp,kd,RR_q,dq,RR_tau);
      // FL_leg->set_leg_disable(3);
      FL_leg->print_pose();
      FR_leg->print_pose();
      RL_leg->print_pose();
      RR_leg->print_pose();
      // std::cout<<"max"<<FR_leg->_leg_motor1->max_pose<<"\tmin"<<FR_leg->_leg_motor1->min_pose<<std::endl;
    }
    

}
#ifndef MOTOR_H
#define MOTOR_H
#include "motor_msg.h"
#include "uart.hpp"
#include <string> // 包含std::string的头文件
#include <memory>
#include <iostream>
#include <algorithm>
class motor
{

public:
    motor(float max_,float min_,std::shared_ptr<Uart> serial = nullptr) 
    : max_pose(max_),
      min_pose(min_),
      serial_(serial)
    {
        if (serial_ == nullptr)
        {
            serial_ = std::make_shared<Uart>("/dev/ttyUSB0");
        }
    };
    ~motor()
    {}
    void motor_enable(int n) // 添加电机id
    {
        cmd.mode = 10;
        cmd.id = n;
    }
    void set_motor(int id, float kp, float kd, float q, float dq, float tau) // 力位混合控制
    {
        cmd.mode = 10;
        cmd.id = id;
        cmd.K_P=std::max(-0.03f, std::min(kp, 0.03f));
        cmd.K_W=std::max(-5.f, std::min(kd, 5.f));
        cmd.T=std::max(-0.5f, std::min(tau, 0.5f));
        cmd.Pos = q;
        cmd.W = dq;
        std::cout<<"cmd.q"<<q<<std::endl;
        if (motor_limit(id)){
            std::cout <<  "motor.cmd: "    << cmd.Pos     <<  std::endl;
            std::cout <<  "motor.dq: " << cmd.W <<  std::endl;
            motor_sendRecv();
        }
    }
    void set_motor_pose(int n, float pose) // 位置模式
    {   
        motor_enable(n);
        cmd.K_P = rotor_kp;
        cmd.K_W = rotor_kd;
        cmd.Pos = pose;
        cmd.W = 0.0;
        cmd.T = 0;
        if (motor_limit(n))
            motor_sendRecv();
    }
    void set_motor_disable(int n, float kd) // 阻尼模式
    {
        cmd.mode = 10;
        cmd.id = n;
        cmd.K_P = 0.;
        cmd.K_W = kd;
        cmd.Pos = 0;
        cmd.W = 0.0;
        cmd.T = 0.0;
        motor_sendRecv();
        // usleep(200);
    }
    void set_motor_torque(int n, float t) // 力矩模式
    {
        cmd.mode = 10;
        cmd.id = n;
        cmd.K_P = 0.;
        cmd.K_W = 0;
        cmd.Pos = 0;
        cmd.W = 0.0;
        cmd.T = t;
        cmd.T=std::max(-0.5f, std::min(cmd.T, 0.5f));
        motor_sendRecv();
    }

public:
    MotorCmd cmd;
    MotorData data;
    std::shared_ptr<Uart> serial_;
    float output_kp = 25;
    float output_kd = 0.6;
    float gear_ratio = 9.1;
    float rotor_kp = (output_kp / (gear_ratio * gear_ratio)) / 26.07;
    float rotor_kd = (output_kd / (gear_ratio * gear_ratio)) * 100.0;
    float max_pose;
    float min_pose;
private:
    bool motor_limit(int n)//限制力矩
    {
        if (max_pose==0||min_pose==0)
        {
            set_motor_torque(n,0.2);
            return false;
        }
        // if(data.q<min_pose+1.5||data.q>max_pose-1.5){
        if(data.Pos<min_pose+0.5||data.Pos>max_pose-0.5){
            if(data.T>1||data.T<-1){
                set_motor_disable(n,3);
                return false;
            }
        }
        return true;
    }
    void motor_sendRecv() // 发送数据
    {
        if (cmd.mode == 10)
        {
            serial_->SendRecv(cmd);
            usleep(200);
            data = serial_->GetMotorData();
            usleep(200);
        }
        else
        {
            std::cout << "Please initialize the motor" << std::endl;
        }
        // usleep(200);
    }
};
#endif
//使用示例
//1.无最大值和最小值时
// int main(){
//     std::shared_ptr<SerialPort> serial_=std::make_shared<SerialPort>("/dev/ttyUSB0");
//     std::shared_ptr<motor> motor_=std::make_shared<motor>(0,0,serial_);

//     while (1)
//     {
//         motor_->set_motor_torque(0,0.2);
//         std::cout<<"motor.q"<<motor_->data.q<<std::endl;
//     }
    
// }
//2.力位混合控制 以小腿为例
// #define calf_max 28.5448f
// #define calf_min 4.52717f
// #define calf_joint 20.f
// #define calf_start_joint 4.f
// int main(){
//     std::shared_ptr<SerialPort> serial_=std::make_shared<SerialPort>("/dev/ttyUSB0");
//     std::shared_ptr<motor> motor_=std::make_shared<motor>(calf_max,calf_min,serial_);
// //   //calf_max calf_min为小腿范围
//     float _percent = 0;
//     float _duration = 10; 
//     while (1)
//     {
//         _percent += (float)1/_duration;
//         _percent = _percent > 1 ? 1 : _percent;
//         float pose = (1 - _percent)*calf_start_joint + _percent*calf_joint;
//         motor_->set_motor(2,0.015,3,pose,7,0.3);
//     }
// }





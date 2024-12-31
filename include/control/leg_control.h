
#ifndef LEG_CONTROL_H
#define LEG_CONTROL_H
#include <vector>
#include "motor.h"
#define PI 3.141592653589793f
#define calf_ammount (PI-0.3628)*9.1
#define thigh_ammount (-PI/2+0.3628)*9.1
class leg_control
{

public:
    leg_control(const std::string& leg_name, const std::string& port)
           : _leg_name(leg_name),
             _leg_serial(std::make_shared<Uart>(port))  // 初始化 SerialPort 对象
    {
      _leg_motor0=std::make_shared<motor>(0,0,1 ,_leg_serial);
      _leg_motor1=std::make_shared<motor>(1,thigh_ammount,1,_leg_serial);
      _leg_motor2=std::make_shared<motor>(2,calf_ammount,1,_leg_serial);
    };
    ~leg_control(){
    };
    void set_motor_cmd(std::vector<float>& kp, std::vector<float>& kd,
std::vector<float>& q, std::vector<float>& dq, std::vector<float>& tau);    //控制腿中的一个电机
    void set_leg_disable(float kd);//阻尼模式
    void set_leg_pose(std::vector<float>& arr);  //位置控制
    void set_leg_torque(std::vector<float>& arr); //力矩控制
    void read_data(); //获得整条腿数据
    std::vector<float> recv_pose(); //获得整条腿数据
public:
    std::string _leg_name;            // 定义腿的名称
    std::shared_ptr<Uart> _leg_serial;  // 定义 SerialPort 对象
    std::shared_ptr<motor> _leg_motor0;           // 定义三个个 Motor 对象
    std::shared_ptr<motor> _leg_motor1; 
    std::shared_ptr<motor> _leg_motor2;
    MotorData leg_data[3];
    std::vector<float> leg_pose[3];
    std::vector<float> leg_speed[3];
};



#endif
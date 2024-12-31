#include "leg_control.h"


void leg_control::set_motor_cmd(std::vector<float>& kp, std::vector<float>& kd,
std::vector<float>& q, std::vector<float>& dq, std::vector<float>& tau){
    _leg_motor0->set_motor(kp[0],kd[0],q[0],dq[0],tau[0]);
    _leg_motor1->set_motor(kp[1],kd[1],q[1],dq[1],tau[1]);
    _leg_motor2->set_motor(kp[2],kd[2],q[2],dq[2],tau[2]);
    read_data();
}
void leg_control::set_leg_disable(float kd){

}
void leg_control::set_leg_pose(std::vector<float>& arr){

}
void leg_control::set_leg_torque(std::vector<float>& arr){

}

void leg_control::read_data(){
    leg_data[(int)_leg_motor0->data.motor_id]=_leg_motor0->data;
    leg_data[(int)_leg_motor1->data.motor_id]=_leg_motor1->data;
    leg_data[(int)_leg_motor2->data.motor_id]=_leg_motor2->data;
}
std::vector<float> leg_control::recv_pose(){
    std::vector<float> arr(3);
    arr[0]=_leg_motor0->recv_pose(leg_data[0].Pos);
    arr[1]=_leg_motor1->recv_pose(leg_data[1].Pos);
    arr[2]=_leg_motor2->recv_pose(leg_data[2].Pos);
    return arr;
}

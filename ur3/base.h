#pragma once
#include<vector>
#include"IK.h"
#define PI 3.1415926535

//与Vrep进行连接通信
int vrep_connect();

//获得Vrep中的对象
void GetVrepObject();

//机械臂到达指定点
void RobotMove(pose TargetPose);

//选择逆解中的最优解
std::vector<double> ChooseTheBest(std::vector<std::vector<double>> allchoose);

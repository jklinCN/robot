#pragma once
#include<vector>
#include"IK.h"
#define PI 3.1415926535

//��Vrep��������ͨ��
int vrep_connect();

//���Vrep�еĶ���
void GetVrepObject();

//��е�۵���ָ����
void RobotMove(pose TargetPose);

//ѡ������е����Ž�
std::vector<double> ChooseTheBest(std::vector<std::vector<double>> allchoose);

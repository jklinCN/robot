#pragma once
#include"IK.h"
extern "C" {
#include"extApi.h"
#include"extApiPlatform.h"
}
#define PI 3.1415926535

//��Vrep��������ͨ��
int vrep_connect();

//���Vrep�еĶ���
void GetVrepObject();

//��е�۵���ָ����
void RobotMove(pose TargetPose);

//ѡ������е����Ž�
std::vector<double> ChooseTheBest(std::vector<std::vector<double>> allchoose);

#pragma once
#include<Eigen/dense>
#include<vector>

//����λ��
typedef struct pose
{
	double x, y, z, RX, RY, RZ;
	pose(double _x, double _y, double _z, double _RX, double _RY, double _RZ) :x(_x), y(_y), z(_z), RX(_RX), RY(_RY), RZ(_RZ) {}
}pose;

//���˶�ѧ������һ��Ƕ�ֵ������һ��4*4����
Eigen::Matrix4d kinematics(std::vector<double> theta_input);

//���˶�ѧ������һ��λ�ˣ����ض�Ӧ�İ����
std::vector<std::vector<double>> Inverse_kinematics(pose t);
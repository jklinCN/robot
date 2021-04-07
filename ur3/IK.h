#pragma once
#include<Eigen/dense>
using namespace Eigen;
//定义位姿
typedef struct pose
{
	double x, y, z, RX, RY, RZ;
	pose(double _x, double _y, double _z, double _RX, double _RY, double _RZ) :x(_x), y(_y), z(_z), RX(_RX), RY(_RY), RZ(_RZ) {}
}pose;

//正运动学，输入一组角度值，返回一个4*4矩阵
Eigen::Matrix4d kinematics(double* theta_input);

//逆运动学，输入一个位姿，返回对应的八组解
double** Inverse_kinematics(pose t);
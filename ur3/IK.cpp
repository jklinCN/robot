#include"IK.h"
#include<iostream>
#include<memory>
#include<Eigen/Dense>
using namespace std;
using namespace Eigen;

//ur3数据
const double d[7] = { 0,0.1519,0,0,0.11235,0.08535,0.0819 };//第0个不用
const double a[6] = { 0,0,-0.24365,-0.21325,0,0 };//a有0，没有6
const double alpha[6] = { 0,90,0,0,90,-90 };//alpha有0，没有6

//测试数据1  -118.43 -268.05 157.28 0.001 -3.166 -0.040
//一组解 -91.71 -98.96 -126.22 -46.29 91.39 358.22

//测试数据2 -63.78 -201.25 137.28 0.192 3.109 0.036
//一组解 -76.28 -83.49 -151.01 -36.32 91.85 20.76

//逆解结果初始化
double** result_init()
{
	double** t;
	t = (double**)malloc(sizeof(double*) * (8 + 1));
	for (int i = 0; i <= 8; i++)
		t[i] = (double*)malloc(sizeof(double) * (6 + 1));
	return t;
}

Eigen::Matrix4d kinematics(double* theta_input)
{
	Eigen::Matrix4d T[6 + 1];//为了和theta对应，0不用
	for (int i = 1; i <= 6; i++)
	{
		T[i](0, 0) = cos(theta_input[i]);
		T[i](0, 1) = -sin(theta_input[i]);
		T[i](0, 2) = 0;
		T[i](0, 3) = a[i - 1];
		T[i](1, 0) = sin(theta_input[i]) * cos(alpha[i - 1] / 180 * EIGEN_PI);
		T[i](1, 1) = cos(theta_input[i]) * cos(alpha[i - 1] / 180 * EIGEN_PI);
		T[i](1, 2) = -sin(alpha[i - 1] / 180 * EIGEN_PI);
		T[i](1, 3) = -sin(alpha[i - 1] / 180 * EIGEN_PI) * d[i];
		T[i](2, 0) = sin(theta_input[i]) * sin(alpha[i - 1] / 180 * EIGEN_PI);
		T[i](2, 1) = cos(theta_input[i]) * sin(alpha[i - 1] / 180 * EIGEN_PI);
		T[i](2, 2) = cos(alpha[i - 1] / 180 * EIGEN_PI);
		T[i](2, 3) = cos(alpha[i - 1] / 180 * EIGEN_PI) * d[i];
		T[i](3, 0) = 0;
		T[i](3, 1) = 0;
		T[i](3, 2) = 0;
		T[i](3, 3) = 1;
	}
	Eigen::Matrix4d T06 = T[1] * T[2] * T[3] * T[4] * T[5] * T[6];
	cout << "检验得：X=" << T06(0, 3) << "    Y=" << T06(1, 3) << "     Z=" << T06(2, 3);
	return T06;
}
double** Inverse_kinematics(pose t)
{
	//输入与输出均单位为m
	double** theta = result_init();

	//1.旋转向量转旋转矩阵
	Eigen::Vector3d v(t.RX, t.RY, t.RZ);
	double t_alpha = v.norm();//求模
	v.normalize();//标准化
	Eigen::AngleAxisd rv(t_alpha, v);//旋转向量
	Eigen::Matrix3d rm;
	rm = rv.matrix();

	//2.求解
	double A, B, C, D, E, F, G, M, N;

	//theta1
	A = rm(0, 2) * d[6] - t.x;
	B = rm(1, 2) * d[6] - t.y;
	C = d[4];
	//第一个解，赋给一到四组
	theta[1][1] = atan2(B, A) - atan2(C, sqrt(A * A + B * B - C * C));
	theta[2][1] = theta[1][1];
	theta[3][1] = theta[1][1];
	theta[4][1] = theta[1][1];
	//第二个解，赋给五到八组
	theta[5][1] = atan2(B, A) - atan2(C, -sqrt(A * A + B * B - C * C));
	theta[6][1] = theta[5][1];
	theta[7][1] = theta[5][1];
	theta[8][1] = theta[5][1];

	//theta5
	//由theta[1][1]产生的第一个解，赋给一到二组
	A = sin(theta[1][1]) * rm(0, 2) - cos(theta[1][1]) * rm(1, 2);
	theta[1][5] = atan2(sqrt(1 - A * A), A);
	theta[2][5] = theta[1][5];
	//由theta[1][1]产生的第二个解，赋给三到四组
	theta[3][5] = atan2(-sqrt(1 - A * A), A);
	theta[4][5] = theta[3][5];
	//由theta[5][1]产生的第一个解，赋给五到六组
	A = sin(theta[5][1]) * rm(0, 2) - cos(theta[5][1]) * rm(1, 2);
	theta[5][5] = atan2(sqrt(1 - A * A), A);
	theta[6][5] = theta[5][5];
	//由theta[5][1]产生的第二个解，赋给七到八组
	theta[7][5] = atan2(-sqrt(1 - A * A), A);
	theta[8][5] = theta[7][5];

	//theta6
	for (int i = 1; i <= 8; i++)
	{
		A = (-sin(theta[i][1]) * rm(0, 1) + cos(theta[i][1]) * rm(1, 1)) / theta[i][5];
		B = (sin(theta[i][1]) * rm(0, 0) - cos(theta[i][1]) * rm(1, 0)) / theta[i][5];
		theta[i][6] = atan2(A, B);
	}

	//theta2、theta3、theta4
	for (int i = 1; i <= 8; i = i + 2)
	{
		//先算theta2+theta3+theta4
		double theta234[8 + 1];
		A = rm(2, 2) / sin(theta[i][5]);
		B = (cos(theta[i][1]) * rm(0, 2) + sin(theta[i][1]) * rm(1, 2)) / sin(theta[i][5]);
		theta234[i] = atan2(A, B) - EIGEN_PI;
		theta234[i + 1] = theta234[i];

		//消去theta2+theta3，计算theta2
		A = -cos(theta234[i]) * sin(theta[i][5]) * d[6] + sin(theta234[i]) * d[5];
		B = -sin(theta234[i]) * sin(theta[i][5]) * d[6] - cos(theta234[i]) * d[5];
		C = cos(theta[i][1]) * t.x + sin(theta[i][1]) * t.y;
		D = t.z - d[1];
		M = C - A;
		N = D - B;
		E = -2 * N * a[2];
		F = 2 * M * a[2];
		G = M * M + N * N + a[2] * a[2] - a[3] * a[3];
		theta[i][2] = atan2(F, E) - atan2(G, sqrt(E * E + F * F - G * G));
		theta[i + 1][2] = atan2(F, E) - atan2(G, -sqrt(E * E + F * F - G * G));

		//用theta2反求theta2+theta3
		double theta23[8 + 1];
		theta23[i] = atan2((N - sin(theta[i][2]) * a[2]) / a[3], (M - cos(theta[i][2]) * a[2]) / a[3]);
		theta23[i + 1] = atan2((N - sin(theta[i + 1][2]) * a[2]) / a[3], (M - cos(theta[i + 1][2]) * a[2]) / a[3]);

		//theta3
		theta[i][3] = theta23[i] - theta[i][2];
		theta[i + 1][3] = theta23[i + 1] - theta[i + 1][2];

		//theta4
		theta[i][4] = theta234[i] - theta23[i];
		theta[i + 1][4] = theta234[i + 1] - theta23[i + 1];
	}
	//输出并检验
	for (int i = 1; i <= 8; i++)
	{
		cout << "第" << i << "组解：" << endl;
		for (int j = 1; j <= 6; j++)
			cout << "theta" << j << "=" << theta[i][j] * 180 / EIGEN_PI << "  ";
		cout << endl;
		kinematics(theta[i]);
		cout << endl << endl;
	}
	return theta;
}
#include "base.h"
#include<iostream>
#include<string>
extern "C" {
#include"extApi.h"
#include"extApiPlatform.h"
}
using namespace std;
extern int client_id;
extern int jointHandles[6];
extern int target;
extern int ur3;
int vrep_connect()
{
	int port = 3000;
	int client_id = simxStart("127.0.0.1", port, true, true, 2000, 5);
	if (client_id != -1)
	{
		printf("V-rep connected.\n\n");
		return client_id;
	}
	else
	{
		printf("V-rep connection failed.\n\n");
		exit(-1);
	}
	//判断连接状态:if (simxGetConnectionId(client_id) != -1)
}

void GetVrepObject()
{
	simxGetObjectHandle(client_id, "UR3", &ur3, simx_opmode_blocking);
	simxGetObjectHandle(client_id, "UR3_connection", &target, simx_opmode_blocking);
	for (int i = 0; i < 6; i++)
	{
		string temp_name = "UR3_joint" + to_string(i + 1);
		simxGetObjectHandle(client_id, temp_name.c_str(), &jointHandles[i], simx_opmode_blocking);
	}
}

vector<double> ChooseTheBest(vector<vector<double>> allchoose)
{
	double min = 999999;
	int min_index = 1;
	for (int i = 1; i <= 8; i++)
	{
		//获得当前角度
		float value;
		double total_gap = 0;
		for (int j = 0; j < 6; j++)
		{
			simxGetJointPosition(client_id, jointHandles[j], &value, simx_opmode_blocking);
			total_gap = total_gap + abs(value - allchoose[i][j + 1]);
		}
		if (total_gap < min)
		{
			min = total_gap;
			min_index = i;
		}
	}
	vector<double> thebest;
	thebest = allchoose[min_index];
	for (int i = 1; i <= 6; i++)
		cout << thebest[i] * 180 / PI << "  ";
	cout << min_index << endl;
	return thebest;
}

void CheckError(pose TargetPose)
{
	float position[3];
	simxGetObjectPosition(client_id, target, -1, position, simx_opmode_blocking);
	//todo
}

void RobotMove(pose TargetPose)
{
	vector<vector<double>> IKSolveResult;
	IKSolveResult = Inverse_kinematics(TargetPose);
	vector<double> thebest = ChooseTheBest(IKSolveResult);
	for (int i = 0; i < 6; i++)
		simxSetJointTargetPosition(client_id, jointHandles[i], thebest[i + 1], simx_opmode_oneshot);
	//CheckError(TargetPose);
}
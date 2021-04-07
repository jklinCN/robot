#include "base.h"
#include<iostream>
#include<string>
extern "C" {
#include"extApi.h"
#include"extApiPlatform.h"
}
using namespace std;
extern int client_id;
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

vector<double> ChooseTheBest(vector<vector<double>> allchoose)
{
	double min = 999999;
	int min_index = 1;
	for (int i = 1; i <= 8; i++)
	{
		//获得当前角度
		int jointHandles[6];
		float value;
		double total_gap = 0;
		for (int j = 0; j < 6; j++)
		{
			string temp_name = "UR3_joint" + to_string(j + 1);
			simxGetObjectHandle(client_id, temp_name.c_str(), &jointHandles[j], simx_opmode_blocking);
		}
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
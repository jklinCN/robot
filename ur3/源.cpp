#include<iostream>
#include<math.h>
#include<string>
extern "C" {
#include"extApi.h"
#include"extApiPlatform.h"
}
#define pi 3.1415926535
using namespace std;
int main()
{
	//---------------------����V-rep-------------------
	int port = 3000;
	int client_id = simxStart("127.0.0.1", port, true, true, 2000, 5);
	if (client_id != -1)
		printf("V-rep connected.\n");
	else
	{
		printf("V-rep connection failed.");
		return -1;
	}
	//�ж�����״̬:if (simxGetConnectionId(client_id) != -1)

	//----------------------��ȡ����-------------------
	int control_handle;
	simxGetObjectHandle(client_id, "UR3", &control_handle, simx_opmode_blocking);
	int jointHandles[7];//����0
	for (int i = 1; i <= 6; i++)
	{
		string temp_name = "UR3_joint" + to_string(i);
		simxGetObjectHandle(client_id, temp_name.c_str(), &jointHandles[i], simx_opmode_blocking);
	}
	//----------------------�������--------------------
	/*
	//--------------------��ȡλ����Ϣ
	float position[4];
	simxGetObjectPosition(client_id, control_handle, -1, position, simx_opmode_blocking);
	printf("(%f,%f,%f)\n", position[0], position[1], position[2]);
	for (int i = 1; i <= 6; i++)
	{
		simxGetObjectPosition(client_id, jointHandles[i], -1, position, simx_opmode_blocking);
		printf("(%f,%f,%f)\n", position[0], position[1], position[2]);
	}
	*/
	//--------------------����Ŀ��Ƕ�------------------
	for (int i = 1; i <= 6; i++)
		simxSetFloatSignal(client_id, ("target_pos" + to_string(i)).c_str(), 90 * pi / 180, simx_opmode_oneshot);
	simxSetFloatSignal(client_id, ("target_pos" + to_string(3)).c_str(), -90 * pi / 180, simx_opmode_oneshot);
	simxSetIntegerSignal(client_id, "flag", 1, simx_opmode_oneshot);
	system("pause");
	simxFinish(client_id);
}
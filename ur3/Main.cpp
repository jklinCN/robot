#include<iostream>
#include<math.h>
#include<string>
#include<windows.h>
extern "C" {
#include"extApi.h"
#include"extApiPlatform.h"
}
#include"IK.h"
#include"base.h"

//һ���������
pose test(63.78 / 1000, 201.25 / 1000, 287.28 / 1000, 0.192, 3.109, 0.036);

//ȫ�ֱ�������
int client_id;
int ur3;
int target;
int jointHandles[6];

int main()
{
	//---------------����V-rep-------------------
	client_id = vrep_connect();
	//---------------��ȡ����-------------------
	GetVrepObject();
	//---------------Զ�̲���-------------------
	RobotMove(test);

	system("pause");
	simxFinish(client_id);
}
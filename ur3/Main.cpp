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

//一组测试数据
pose test(63.78 / 1000, 201.25 / 1000, 287.28 / 1000, 0.192, 3.109, 0.036);

//全局变量定义
int client_id;
int ur3;
int target;
int jointHandles[6];

int main()
{
	//---------------连接V-rep-------------------
	client_id = vrep_connect();
	//---------------获取对象-------------------
	GetVrepObject();
	//---------------远程操作-------------------
	RobotMove(test);

	system("pause");
	simxFinish(client_id);
}
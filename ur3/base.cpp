#include "base.h"
#include<iostream>
extern "C" {
#include"extApi.h"
#include"extApiPlatform.h"
}
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
	//ÅÐ¶ÏÁ¬½Ó×´Ì¬:if (simxGetConnectionId(client_id) != -1)
}
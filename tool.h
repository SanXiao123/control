#pragma once
#include <iostream>
using namespace std;
#define pi acos(-1)

//定义路径点
typedef struct waypoint {
	int ID;
	double x, y, yaw, K;//x,y,yaw,曲率K
}waypoint;

//定义小车状态
typedef struct vehicleState {
	double x, y, yaw, v, kesi;//x,y,yaw,前轮偏角kesi
}vehicleState;

//定义控制量
typedef struct U {
	double v;
	double kesi;//速度v,前轮偏角kesi
}U;

double normalize_angle(double angle);//角度归一化 [-pi,pi];

double limit_kesi(double kesi);//前轮转角限幅 [-pi/2,pi/2];


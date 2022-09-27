#include <iostream>
#include <vector>
#include <trajectory.h>
#include <math.h>
using namespace std;

void trajectory::refer_path() {
	waypoint PP;
	for (int i = 0; i < 1000; i++)
	{
		PP.ID = i;
		PP.x = 0.1 * i;//x
		PP.y = 2.0 * sin(PP.x / 5.0) + 2.0 * cos(PP.x / 2.5);//y
		//直线
		//PP.y = 5.5;
		PP.yaw = PP.K = 0.0;
		waypoints.push_back(PP);
	}

	for (int j = 0; j < waypoints.size(); j++) {
		//差分法求一阶导和二阶导
		double dx, dy, ddx, ddy;
		if (j == 0) {
			dx = waypoints[1].x - waypoints[0].x;
			dy = waypoints[1].y - waypoints[0].y;
			ddx = waypoints[2].x + waypoints[0].x - 2 * waypoints[1].x;
			ddy = waypoints[2].y + waypoints[0].y - 2 * waypoints[1].y;
		}
		else if (j == (waypoints.size() - 1)) {
			dx = waypoints[j].x - waypoints[j - 1].x;
			dy = waypoints[j].y - waypoints[j - 1].y;
			ddx = waypoints[j].x + waypoints[j - 2].x - 2 * waypoints[j].x;
			ddy = waypoints[j].y + waypoints[j - 2].y - 2 * waypoints[j].y;
		}
		else {
			dx = waypoints[j + 1].x - waypoints[j].x;
			dy = waypoints[j + 1].y - waypoints[j].y;
			ddx = waypoints[j + 1].x + waypoints[j - 1].x - 2 * waypoints[j].x;
			ddy = waypoints[j + 1].y + waypoints[j - 1].y - 2 * waypoints[j].y;
		}
		waypoints[j].yaw = atan2(dy, dx);//yaw
		//计算曲率：设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
		double AAA = sqrt(pow((pow(dx, 2) + pow(dy, 2)), 3));
		waypoints[j].K = (ddy * dx - ddx * dy) / (sqrt(pow((pow(dx, 2) + pow(dy, 2)), 3)));
	}
}

vector<waypoint> trajectory::get_path() {
	return waypoints;
}

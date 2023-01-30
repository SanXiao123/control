//                            _ooOoo_  
//                           o8888888o  
//                           88" . "88  
//                           (| -_- |)  
//                            O\ = /O  
//                        ____/`---'\____  
//                      .   ' \\| |// `.  
//                       / \\||| : |||// \  
//                     / _||||| -:- |||||- \  
//                       | | \\\ - /// | |  
//                     | \_| ''\---/'' | |  
//                      \ .-\__ `-` ___/-. /  
//                   ___`. .' /--.--\ `. . __  
//                ."" '< `.___\_<|>_/___.' >'"".  
//               | | : `- \`.;`\ _ /`;.`/ - ` : | |  
//                 \ \ `-. \_ __\ /__ _/ .-` / /  
//         ======`-.____`-.___\_____/___.-`____.-'======  
//                            `=---='  
//  
//         .............................................  
//                  佛祖保佑             永无BUG

//          佛曰:  
//                  写字楼里写字间，写字间里程序员；  
//                  程序人员写程序，又拿程序换酒钱。  
//                  酒醒只在网上坐，酒醉还来网下眠；  
//                  酒醉酒醒日复日，网上网下年复年。  
//                  但愿老死电脑间，不愿鞠躬老板前；  
//                  奔驰宝马贵者趣，公交自行程序员。  
//                  别人笑我忒疯癫，我笑自己命太贱；  
//                  不见满街漂亮妹，哪个归得程序员？

#include <iostream>
#include <LQR.h>
#include <vector>
#include <trajectory.h>
#include <stdlib.h>
#include "matplotlibcpp.h"
using namespace std;
namespace plt = matplotlibcpp;

#define pi acos(-1)
#define T 0.05//采样时间   很有意思的测试数据：T=0.5s，允许误差范围为±5.0m；T=0.1s，允许误差范围为±10.0m；T=0.05s；允许误差范围为±11m
#define L 0.5//车辆轴距
#define V_DESIRED 0.5//期望速度

vehicleState update_state(U control, vehicleState car) {
	car.v = control.v;
	car.kesi = control.kesi;
	car.x += car.v * cos(car.yaw) * T;
	car.y += car.v * sin(car.yaw) * T;
	car.yaw += car.v / L * tan(car.kesi) * T;
	//car.yaw = normalize_angle(car.yaw);
	return car;
}

class Path {
private:
	vector<waypoint> path;
public:
	//添加新的路径点
	void Add_new_point(waypoint& p)
	{
		path.push_back(p);
	}

	void Add_new_point(vector<waypoint>& p) 
	{
		path = p;
	}

	//路径点个数
	unsigned int Size()
	{
		return path.size();
	}

	//获取路径点
	waypoint Get_waypoint(int index)
	{
		waypoint p;
		p.ID = path[index].ID;
		p.x = path[index].x;
		p.y = path[index].y;
		p.yaw = path[index].yaw;
		p.K = path[index].K;
		return p;
	}


	// 搜索路径点, 将小车到起始点的距离与小车到每一个点的距离对比，找出最近的目标点索引值
	int Find_target_index(vehicleState state)
	{
		double min = abs(sqrt(pow(state.x - path[0].x, 2) + pow(state.y - path[0].y, 2)));
		int index = 0;
		for (int i = 0; i < path.size(); i++)
		{
			double d = abs(sqrt(pow(state.x - path[i].x, 2) + pow(state.y - path[i].y, 2)));
			if (d < min)
			{
				min = d;
				index = i;
			}
		}

		//索引到终点前，当（机器人与下一个目标点的距离Lf）小于（当前目标点到下一个目标点距离L)时，索引下一个目标点
		if ((index + 1) < path.size())
		{
			double current_x = path[index].x; double current_y = path[index].y;
			double next_x = path[index + 1].x; double next_y = path[index + 1].y;
			double L_ = abs(sqrt(pow(next_x - current_x, 2) + pow(next_y - current_y, 2)));
			double L_1 = abs(sqrt(pow(state.x - next_x, 2) + pow(state.y - next_y, 2)));
			//ROS_INFO("L is %f,Lf is %f",L,Lf);
			if (L_1 < L_)
			{
				index += 1;
			}
		}
		return index;
	}

};

class LQR_node {
private:
	vehicleState car;//小车状态
	double Q[3];
	double R[2];
	int lastIndex;//最后一个点索引值
	waypoint lastPoint;//最后一个点信息
	vector<double> x,y,x_p,y_p,v_a,v_d,kesi_a,kesi_d;

public:
	LQR* controller = new LQR();
	Path* path = new Path();
	trajectory* trajec = new trajectory();

	LQR_node()//初始化中添加轨迹、小车初始位姿
	{
		//ROS:
		addpointcallback();
		//robot:
		car.x = -1.325;
		car.y = 2.562;
		car.yaw = 0.964;
		car.v = 0.0;
		car.kesi = 0.1;
		
	}

	~LQR_node() {
		free(controller);
		free(path);
		free(trajec);
	}

	void addpointcallback(){
		trajec->refer_path();
		vector<waypoint> waypoints = trajec->get_path();
		path->Add_new_point(waypoints);
		cout << "path size is:" << path->Size() << endl;
		lastIndex = path->Size() - 1;
		lastPoint = path->Get_waypoint(lastIndex);
	}

	double slow_judge(double distance) {
		if (distance>=5.0&&distance <= 15.0) {
			return 0.35;
		}
		else if (distance>=0.1&&distance < 5.0) {
			return 0.15;
		}
		else if (distance < 0.1) {
			printf("reach goal!\n");
			plot_();
		}
		else
		{
			return V_DESIRED;
		}
	}

	//控制器流程
	void LQR_track() {
		U U_r;
		waypoint Point;

		//搜索路径点
		int target_index = path->Find_target_index(car);
		printf("target index is : %d\n", target_index);

		//获取路径点信息，构造期望控制量
		Point = path->Get_waypoint(target_index);
		//printf("waypoint information is x:%f,y:%f,yaw:%f,K:%f\n", Point.x, Point.y, Point.yaw, Point.K);
		
		//减速判断
		double kesi = atan2(L * Point.K, 1);
		double v_distance = abs(sqrt(pow(car.x - lastPoint.x, 2) + pow(car.y - lastPoint.y, 2)));
		printf("the distance is %f\n", v_distance);
		U_r.v = slow_judge(v_distance);U_r.kesi = kesi;
		printf("the desired v is: %f,the desired kesi is: %f\n", U_r.v,U_r.kesi);

		//权重矩阵
		Q[0] = 1.0; Q[1] = 1.0; Q[2] = 1.0;
		R[0] = 4.0; R[1] = 4.0;

		//使用LQR控制器
		controller->initial(L, T, car, Point, U_r, Q, R);//初始化控制器
		U control = controller->cal_vel();//计算输入[v, kesi]
		printf("the speed is: %f,the kesi is: %f\n", control.v, control.kesi);
		printf("the car position is x: %f, y: %f\n", car.x, car.y);

		//储存小车位姿用来画图
		x.push_back(car.x);
		y.push_back(car.y);
		v_a.push_back(car.v);
		v_d.push_back(U_r.v);
		kesi_a.push_back(car.kesi);
		kesi_d.push_back(U_r.kesi);
		
		//小车位姿状态更新
		car = update_state(control, car);
	}

	//控制启停函数
	void control() {
		int i = 0;
		
		while (i < 10000) {
			LQR_track();
			i++;
		}
		
	}

	//画图程序
	void plot_() {
		vector<double> time;
		for (int i = 0; i < path->Size(); i++) {
			x_p.push_back(path->Get_waypoint(i).x);
			y_p.push_back(path->Get_waypoint(i).y);
		}
		for (int j = 0; j < v_a.size(); j++) {
			time.push_back(double(j));
		}

		
		plt::subplot(3, 1, 1);
		plt::title("Car position");
		plt::plot(x_p, y_p, "-k", x, y, "-.r");
		plt::subplot(3, 1, 2);
		plt::title("Car speed");
		plt::plot(time, v_d, "-k", time, v_a, "-.r");
		plt::subplot(3, 1, 3);
		plt::title("Car kesi");
		plt::plot(time, kesi_d, "-k", time, kesi_a, "-.r");
		
		plt::show();
		exit(0);
	}


};

int main(char argc, char* argv) {
	LQR_node* node = new LQR_node();
	node->control();
	return 0;
}



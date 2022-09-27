#include <iostream>
#include <vector>
#include "tool.h"
using namespace std;



class trajectory {
private:
	vector<waypoint> waypoints;

public:
	//set reference trajectory
	void refer_path();
	vector<waypoint> get_path();

};

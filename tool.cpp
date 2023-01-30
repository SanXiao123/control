#include<iostream>
#include<tool.h>

double normalize_angle(double angle)//½Ç¶È¹éÒ»»¯ [-pi,pi];
{
	if (angle > pi) {
		angle -= 2.0 * pi;
	}
	if (angle <= -pi) {
		angle += 2.0 * pi;
	}
	return angle;
}

double limit_kesi(double kesi) {
	if (kesi > pi / 2) {
		kesi = pi / 2;
	}
	if (kesi < -pi / 2) {
		kesi = -pi / 2;
	}
	return kesi;
}

#pragma once
#include<Aria.h>
#include<algorithm>
#include<iostream>
#include<fstream>
class bbox: public ArAction
{
public:
	bbox();
	std::ofstream myreader;
	double graph[500][500];
	double leftDistance{ 0 };
	double rightDistance{ 0 };
	double leftAngle_deg{ 0 };
	double rightAngle_deg{ 0 };
	double probability();
	double robotX{ 0 };
	double robotY{ 0 };
	double robotTh{ 0 };
	double robotRadius{ 300 };
	double halfAngle{ 15 };
	virtual ArActionDesired * fire(ArActionDesired d);
	ArActionDesired desiredState;
	double radians(double a);
	double degrees(double c);
	~bbox();
};


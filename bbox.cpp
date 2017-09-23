#include "bbox.h"



bbox::bbox():ArAction ("bbox")
{
	myreader.open("polarmap.txt");
}


bbox::~bbox()
{
}

double bbox::probability() {


	return 1.0f;
}

 ArActionDesired * bbox::fire(ArActionDesired d) {
	 d.reset();
	 leftDistance= myRobot->checkRangeDevicesCurrentPolar(0.0, 90.0, &leftAngle_deg);
	 rightDistance = myRobot->checkRangeDevicesCurrentPolar(-90.0, 0.0, &rightAngle_deg);


	 robotX = myRobot->getX();
	 robotY = myRobot->getY();
	 robotTh = myRobot->getTh();

	 double X1 = robotX + robotRadius;
	 double Y1 = robotY + robotRadius;

	 double lowerAngle_deg = leftAngle_deg - halfAngle;
	 double upperAngle_deg = leftAngle_deg + halfAngle;

	 double lowerAngle_rad = radians(lowerAngle_deg);
	 double upperAngle_rad = radians(upperAngle_deg);

	 double X2 = leftDistance*cos(lowerAngle_rad)+X1;
	 double Y2 = leftDistance*sin(lowerAngle_rad) + Y1;

	 double X4 = leftDistance*cos(upperAngle_rad) + X1;
	 double Y4 = leftDistance*sin(upperAngle_rad) + Y1;

	 double halfAngle_rad = radians(halfAngle);
	 double d2 = leftDistance / cos(halfAngle_rad);

	 double leftAngle_rad = radians(leftAngle_deg);
	 double X3 = d2*cos(leftAngle_rad) + X1;
	 double Y3 = d2*sin(leftAngle_rad);

	 double xlower = min(X1, X2, X3, X4);
	 double ylower = min(Y1, Y2, Y3, Y4);

	 double xupper = max(X1, X2, X3, X4);
	 double yupper = max(Y1, Y2, Y3, Y4);

	 int x_low = xlower;
	 int y_low = ylower;

	 int x_upp = xupper;
	 int y_upp = yupper;

	 //bounding box made 
	 for (int a = y_low; a <= y_upp; ++a) {

		 for (int b = x_low; b <= x_upp; ++b) {
			 //develop bounding box algorithm here
			 double radius_x = sqrt(pow(b - x_low, 2) + pow(a - y_low, 2));
			 double radius_adj = b - x_low;

			 double angle_rad = acos(radius_adj / radius_x);
			 double angle_deg = radians(angle_rad);
			 double angle_dev = abs(angle_deg - leftAngle_deg);

			 double angle_min = leftAngle_deg + halfAngle;
			 double angle_max = leftAngle_deg - halfAngle;

			 double r_max = radius_x*1.1;
			 double r_min = radius_x*0.9;

			 if (radius_x < r_min && angle_deg<angle_max && angle_deg>angle_min) {

				 double p = 1 - (((leftDistance - radius_x) / leftDistance) + ((halfAngle - angle_dev) / halfAngle)) / 2 - 0.5;
				 //
				 double xleft = cos(leftAngle_deg*(3.141 / 180))*(radius_x + 300); //check this
				 double yleft = sin(leftAngle_deg*(3.141 / 180))*(radius_x + 300);

				 double xleftone = xleft*(cos(robotTh*(3.141 / 180)) - yleft*sin(robotTh*(3.141 / 180)));
				 double yleftone = xleft*(sin(robotTh*(3.141 / 180)) + yleft*cos(robotTh*(3.141 / 180)));

				 double xlefttwo = (robotX + xleftone) / 4.5;
				 double ylefttwo = (robotY + yleftone) / 4.5;

				 int testa = xlefttwo;
				 int testb = ylefttwo;

				 double buffer = graph[testa][testb]; //re check this

				 double fin = (p*buffer) / (p*buffer + ((1 - p)*(1 - buffer)));
				 graph[testa][testb] = fin;

				 myreader << testa << "\t" << testb << "\t" << fin;



			 }
			 else if (radius_x>r_min && radius_x < r_max && angle_deg<angle_max && angle_deg>angle_min) {

				 double p = ((((leftDistance - radius_x) / leftDistance) + ((halfAngle - angle_dev) / halfAngle)) / 2)*0.98 + 0.5;

				 double xleft = cos(leftAngle_deg*(3.141 / 180))*(radius_x + 300); //check this apply radius global here 
				 double yleft = sin(leftAngle_deg*(3.141 / 180))*(radius_x + 300);

				 double xleftone = xleft*(cos(robotTh*(3.141 / 180)) - yleft*sin(robotTh*(3.141 / 180)));
				 double yleftone = xleft*(sin(robotTh*(3.141 / 180)) + yleft*cos(robotTh*(3.141 / 180)));

				 double xlefttwo = (robotX + xleftone) / 4.5;
				 double ylefttwo = (robotY + yleftone) / 4.5;

				 int testa = xlefttwo;
				 int testb = ylefttwo;

				 double buffer = graph[testa][testb]; //re check this

				 double fin = (p*buffer) / (p*buffer + ((1 - p)*(1 - buffer)));
				 graph[testa][testb] = fin;
				 myreader << testa << "\t" << testb << "\t" << fin;

			 }
			 else
				 double p = 0.5;
		 }
	 }

	return &d;
}

 double bbox::degrees(double a) {

	 return (a*(180 / 3.142));
 }
 double bbox::radians(double b) {


	 return (b*(3.142 / 180));
 }

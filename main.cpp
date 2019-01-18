#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include "CubicSpline.h"

int main()
{

	CubicSpline cubicSpline;
	points control;
	point pointA(52.814461,49.872199);	
	point pointB(92.408099,50.044726);
	point pointC(92.359442,15.599696);
	point pointD(61.002686,17.447330);
	control.push_back(pointA);
	control.push_back(pointB);
	control.push_back(pointC);
	control.push_back(pointD);
	cubicSpline.setControlPoints(control);
	SplinePos sPos = cubicSpline.getCurrentPos();
	for (int i = 1; i < 30; ++i)
	{
		SplinePos outsPos;
		if(cubicSpline.forwardOffset(sPos, 5*i, outsPos))
		{
			point point;
			cubicSpline.getPoint(outsPos.Idx, outsPos.T, point);
			printf("==========================offset:%d => x:%f y:%f, heading:%f curvature:%f\n", 5*i, point.x, point.y, point.heading, point.curvature);
		}
		else
		{
			break;
		}		
	}

	// if(cubicSpline.forwardOffset(sPos, 150, outsPos))
	// {
	// 	point point;
	// 	cubicSpline.getPoint(outsPos.Idx, outsPos.T, point);
	// 	printf("===========================> x:%f y:%f, heading:%f curvature:%f\n",  point.x, point.y, point.heading, point.curvature);
	// }

    return 0;
}
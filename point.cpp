#include"point.h"
#include <cmath>


point::point(double ix,double iy, double iheading , double icurvature)
{
	x = ix;
	y = iy;
	heading = iheading;
	curvature = icurvature;
}

void point::set(double ix,double iy, double iheading, double icurvature)
{
	x = ix;
	y = iy;
	heading = iheading;
	curvature = icurvature;	
}

void point::setHeading(double iheading)
{
	heading = iheading;
}

void point::setCurvature(double icurvature)
{
	curvature = icurvature;
}

void point::setPos(double ix,double iy)
{
	x = ix;
	y = iy;
}




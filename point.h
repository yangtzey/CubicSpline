#ifndef POINT_H
#define POINT_H

#include<vector>


class point
{
public:	
	point():
	x(0.0),
	y(0.0),
	heading(0.0),
	curvature(0.0)
	{
	}

	point(double ix,double iy, double iheading = 0.0 , double icurvature = 0.0);

	void set(double ix,double iy, double iheading = 0.0 , double icurvature = 0.0);

	void setHeading(double iheading);

	void setCurvature(double icurvature);

	void setPos(double ix,double iy);

	double x;
	double y;
	double heading;
	double curvature;

};

typedef std::vector<point> points;

#endif 

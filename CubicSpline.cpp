#include"CubicSpline.h"
#include<cmath>
#include <stdio.h>

#define TdeltaDistance 0.05
#define IntegraldeltaN 10

#define PI 3.1415926

#define rad2deg(__rad__) (__rad__*180.0/PI)

CubicSpline::CubicSpline()
: m_SplinePos()
, m_points()
{
	Initialization();	
}

CubicSpline::~CubicSpline()
{

}

void CubicSpline::Initialization()
{
	m_a0 = 0;
	m_a1 = 0;
	m_a2 = 0;
	m_a3 = 0;
	m_b0 = 0;
	m_b1 = 0;
	m_b2 = 0;
	m_b3 = 0;
	m_LastIdx = -1;
	m_points.clear();
	m_SplinePos.cIdx = 1;
}

bool CubicSpline::setControlPoints(points points)
{
	if (points.size() < 4)
	{
		return false;
	}
	Initialization();
	point controlstart(2*points[0].x - points[1].x, 2*points[0].y - points[1].y, 0, 0);
	point controlend(2*points[points.size() - 1].x - points[points.size() - 2].x, 2*points[points.size() - 1].y - points[points.size() - 2].y, 0, 0);
	m_points.push_back(controlstart);
	for (int i = 0; i < points.size(); ++i)
	{
		m_points.push_back(points[i]);
	}
	m_points.push_back(controlend);
	for (int i = 0; i < m_points.size(); ++i)
	{
		printf("control point:  x:%f y:%f \n", 10*i, m_points[i].x, m_points[i].y);
	}
	
	return true;
}

void CubicSpline::clearControlPoints()
{
	m_points.clear();		
	Initialization();
}

bool CubicSpline::makeGeneratingPoint(int idx)
{
	if ((idx > m_points.size() - 4) 
		|| (idx < 0))
	{
		return false;
	}
	if (m_LastIdx != idx)
	{
		if (m_LastIdx > m_points.size() - 4 && m_LastIdx != -1)
		{
			return false;
		}
		points points;
		points.push_back(m_points[idx]);
		points.push_back(m_points[idx+1]);
		points.push_back(m_points[idx+2]);
		points.push_back(m_points[idx+3]);	
		GeneratingPoint(points);
	}

	m_LastIdx = idx;
	return true;
	
}

void CubicSpline::GeneratingPoint(points points)
{
	if (points.size() != 4)
	{
		return;
	}

	double x0 = points[0].x;
	double x1 = points[1].x;
	double x2 = points[2].x;
	double x3 = points[3].x;
	m_a0 = (x0 + 4*x1 +x2)/6;
	m_a1 = -(x0 - x2)/2;
	m_a2 = (x0 - 2*x1 + x2)/2;
	m_a3 = -(x0 - 3*x1 + 3*x2 - x3)/6;

	double y0 = points[0].y;
	double y1 = points[1].y;
	double y2 = points[2].y;
	double y3 = points[3].y;
	m_b0 = (y0 + 4*y1 +y2)/6;
	m_b1 = -(y0 - y2)/2;
	m_b2 = (y0 - 2*y1 + y2)/2;
	m_b3 = -(y0 - 3*y1 + 3*y2 - y3)/6;
}

double CubicSpline::funcX(double t)
{
	return (m_a0 + m_a1*t + m_a2*t*t + m_a3*t*t*t);
}

double CubicSpline::funcY(double t)
{
	return (m_b0 + m_b1*t + m_b2*t*t + m_b3*t*t*t);
}

double CubicSpline::funcK(double x, double y, double t)
{
	double rad = 0;
	double k = ((m_b1+m_b2*t+m_b3*t*t)/(m_a1+m_a2*t+m_a3*t*t));

	point from;
	point to;
	double fromT = 0;
	double toT = 0;

	if (t + 0.05 <= 1)
	{
		fromT = t;
		toT = t + 0.05;
		from.x = x;
		from.y = y;
		to.x = funcX(toT);
		to.y = funcY(toT);
	}
	else if (t - 0.05 >= 0)
	{
		fromT = t - 0.05;
		toT = t;
		from.x = funcX(fromT);
		from.y = funcY(fromT);
		to.x = x;
		to.y = y;
	}
	else
	{

	}
	
	rad = atan(k);

	double heading = rad2deg(rad);
	double Theading = rad2deg(rad);

	if (k > 0)
	{
		if (fabs(Theading) > 45)
		{
			if (from.y > to.y)
			{
				heading = heading - 180;
			}
		}
		else
		{
			if (from.x > to.x)
			{
				heading = heading - 180;
			}
		}		
	}
	else
	{
		if (fabs(Theading) > 45)
		{
			if (from.y < to.y)
			{
				heading = heading + 180;
			}
		}
		else
		{
			if (from.x > to.x)
			{
				heading = heading + 180;
			}
		}		
	}

	heading = 90 - heading;

	if (heading < 0)
	{
		heading = heading + 360;
	}
	if (heading > 360)
	{
		heading = heading - 360;
	}

	return heading;
}

double CubicSpline::funcCurvature(double t)
{
	double Xderivative = m_a3*t*t*3.0 + m_a2*t*2.0 + m_a1;
	double Yderivative = m_b3*t*t*3.0 + m_b2*t*2.0 + m_b1;
	double X2derivative = m_a3*t*6.0 + m_a2*2.0;
	double Y2derivative = m_b3*t*6.0 + m_b2*2.0;
	double Kx = Xderivative*Y2derivative - Yderivative*X2derivative;
	double Ktempy = Xderivative*Xderivative + Yderivative*Yderivative;
	double Ky =  sqrt(Ktempy*Ktempy*Ktempy);
	return Kx/Ky;
}

double CubicSpline::integralX(double min, double max, int N)
{
	double result = 0;
	double delta = (max - min) / N;
	for (double i = min+delta; i < max; i+=delta)
	{
		result += funcX(min+i)*delta;
	}
	return result;
}

double CubicSpline::integralY(double min, double max, int N)
{
	double result = 0;
	double delta = (max - min) / N;
	for (double i = min+delta; i < max; i+=delta)
	{
		result += funcY(min+i)*delta;
	}
	return result;
}

double CubicSpline::getPosDistance(int idx)
{
	double distance = getDistance(m_points[idx],m_points[idx+1]);
	return distance;
}

double CubicSpline::getDistance(point lpoint,point rpoint)
{
	double detlaX = lpoint.x - rpoint.x;
	double detlaY = lpoint.y - rpoint.y;
	double currentDistance = sqrt(detlaX*detlaX + detlaY*detlaY);
	return currentDistance;
}


bool CubicSpline::forwardOffset(SplinePos iPos, double offset, SplinePos& oPos)
{

	double currentLen = getPosDistance(iPos.cIdx);
	if (!makeGeneratingPoint(iPos.Idx))
	{
		return false;
	}

	double detlaT = TdeltaDistance/currentLen;
	double T = iPos.T;
	double currentOffset = 0;

	while(true)
	{
		if (T + detlaT > 1)
		{	
			iPos.cIdx++;
			iPos.Idx++;
			iPos.T = 0;

			return forwardOffset(iPos, offset - currentOffset, oPos);
					
		}

		currentOffset += integralDistance(T, T + detlaT, IntegraldeltaN);

		if(currentOffset > offset)
		{
			break;
		}

		T += detlaT;
	}
	oPos.cIdx = iPos.cIdx;
	oPos.Idx  = iPos.Idx;
	oPos.T = T;
	return true;
}

bool CubicSpline::backwardOffset(SplinePos iPos, double offset, SplinePos& oPos)
{
	double currentLen = getPosDistance(iPos.cIdx);
	if (!makeGeneratingPoint(iPos.Idx))
	{
		return false;
	}

	double detlaT = TdeltaDistance/currentLen;
	double T = iPos.T;
	double currentOffset = 0;
	while(true)
	{
		if (T - detlaT < 0 )
		{	
			iPos.cIdx--;
			iPos.Idx--;
			iPos.T = 1;
			return backwardOffset(iPos, offset - currentOffset, oPos);
		}

		currentOffset += integralDistance(T - detlaT, T, IntegraldeltaN);
		if(currentOffset > offset)
		{
			break;
		}

		T -= detlaT;
	}
	oPos.cIdx = iPos.cIdx;
	oPos.Idx  = iPos.Idx;
	oPos.T = T;
	return true;
}

double CubicSpline::integralDistance(double min, double max, int N)
{
	double distance = 0;
	double delta = (max - min) / N;
	double lastX = funcX(min);
	double lastY = funcY(min);
	for (double i = min+delta; i <= max; i+=delta)
	{
		double currentX = funcX(i);
		double currentY = funcY(i);
		double deltaX = lastX - currentX;
		double deltaY = lastY - currentY;
		distance += sqrt(deltaX*deltaX + deltaY*deltaY);
		lastX = currentX;
		lastY = currentY;
	}
	return distance;
}


bool CubicSpline::getPoint(int idx, double t, point& point)
{
	if ((idx > m_points.size() - 4)
			|| t > 1)
	{
		return false;
	}

	makeGeneratingPoint(idx);
	point.x = funcX(t);
	point.y = funcY(t);
	point.heading = funcK(point.x, point.y, t);
	point.curvature = funcCurvature(t);
	return true;
}

SplinePos CubicSpline::getCurrentPos()
{
	return m_SplinePos;
}


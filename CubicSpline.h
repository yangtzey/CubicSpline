#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include"point.h"

typedef struct SplinePos
{
	SplinePos()
	{
		Idx  = 0;
		T    = 0;
		cIdx = 0;
	}
	int    Idx;
	double T;
	int    cIdx;
}SplinePos;


class CubicSpline
{
public:
	CubicSpline();
	virtual ~CubicSpline();
	void Initialization();
	bool setControlPoints(points points);
	void clearControlPoints();

	void GeneratingPoint(points points);
	bool makeGeneratingPoint(int idx);


	double getDistance(point lpoint,point rpoint);
	double getPosDistance(int idx);

	bool forwardOffset(SplinePos iPos, double offset, SplinePos& oPos);
	bool backwardOffset(SplinePos iPos, double offset, SplinePos& oPos);
	
	bool getPoint(int idx, double t, point& point);
	SplinePos getCurrentPos();


private:

	double funcX(double t);
	double funcY(double t);
	double funcK(double x, double y, double t);
	double integralX(double min, double max, int N);
	double integralY(double min, double max, int N);
	double integralDistance(double min, double max, int N);
	double getVertexDistance(int idx);
	double funcCurvature(double t);

	points m_points;
	double m_T;
	double m_a0;
	double m_a1;
	double m_a2;
	double m_a3;
	double m_b0;
	double m_b1;
	double m_b2;
	double m_b3;

	int m_LastIdx;

	SplinePos m_SplinePos;
	point m_LastPos;
	point m_LastPoint;

};

#endif

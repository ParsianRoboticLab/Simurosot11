//the methods used here all refer to nature coordinate!
#include "stdafx.h"
#include "globe.h"
#include "geometry.h"

/*********** chen ning *******************/
int CalcuFormat(POINT P2, POINT P1, Formulation *Param)
{//----------- calculate the line P1 and P2's formulation according to
	//the format aY = bX + c, a=x1-x2,b=y1-y2, c=x1*y2-x2*y1.
	// return 1 if (x2-x1) > 0, return -1 if (x2-x1) < 0
	// else return 0. the returned value can be used to determine 
	// the line's normal direction.
	//nextstep:aY + bX + c = 0, a = x1-x2,b=y2-y1,c=x2*y1-x1*y2. 
	Param->a = P2.x - P1.x;
	Param->b = P2.y - P2.y;
	Param->c = P1.y*P2.x - P2.y*P1.x;
	if(Param->a > 0)
		return 1;
	if(Param->a < 0)
		return -1;
	return 0;
};

int StdLineForm(dbPOINT P1, dbPOINT P2, Formulation *FormuParam)
{//---------aX + bY + c = 0, b = x1-x2,a=y2-y1,c=x2*y1-x1*y2
	FormuParam->a = P2.y - P1.y;
	FormuParam->b = P1.x - P2.x;
	FormuParam->c = P2.x*P1.y - P1.x*P2.y;
	return 0;
}

int StdLineForm(dbPOINT point, double angle, LINEFORMULATION *Result)
{
//点斜式直线求出标准方程
	if((angle - PI/2) == 0)
	{
		Result->a = 1;
		Result->b = 0;
		Result->c = -point.x;
	}
	else
	{
		Result->a = -tan(angle);
		Result->b = 1;
		Result->c = tan(angle)*point.x - point.y;
	}
	return 1;
}


int StdLineForm(POINT P1, POINT P2, Formulation *FormuParam)
{//---------aX + bY + c = 0, b = x1-x2,a=y2-y1,c=x2*y1-x1*y2
	FormuParam->a = P2.y - P1.y;
	FormuParam->b = P1.x - P2.x;
	FormuParam->c = P2.x*P1.y - P1.x*P2.y;
	return 0;
}
int cn_2LinesCrossPoint(Formulation *Line1, Formulation *Line2, dbPOINT *Result)
{//get Line1 and Line2's cross point, return -1 if no cross point, otherwise return 1.
	double dt = Line1->a*Line2->b - Line2->a*Line1->b;
	if( dt == 0)
		return -1;
	if(Result!=NULL)
	{
		Result->x = (Line1->b*Line2->c - Line2->b*Line1->c)/dt;
		Result->y = (Line1->c*Line2->a - Line2->c*Line1->a)/dt;
	}
	return 1;
}

int cn_2LinesCrossPoint(dbPOINT A1, dbPOINT A2, dbPOINT B1, dbPOINT B2, dbPOINT *Result)
{
	Formulation lineA, lineB;
	StdLineForm(A1, A2, &lineA);
	StdLineForm(B1, B2, &lineB);
	return cn_2LinesCrossPoint(&lineA, &lineB, Result);
}



double cn_LineAngle(LINEFORMULATION *pLine)
{
	double theta = atan2(pLine->a, -pLine->b);
	return cn_AngleTrimPI(theta);
}


int cn_IsPointInSegment(dbPOINT Pt, dbPOINT A, dbPOINT B)
{
	double d1, d2;
	d1 = cn_2PointsDist(Pt, A);
	d2 = cn_2PointsDist(Pt, B);
	if(fabs(d1 + d2 - cn_2PointsDist(A, B)) < 0.0000000001)
		return 1;
	return -1;
}


double cn_LineAngle(dbPOINT A, dbPOINT B)
{// Calculate the angle of line AB and the x-axis, the result is between 0-2*pi
// and the line's direction is from A to B.
	double cn_pi = acos(-1.0);
	double angle=atan2(B.y-A.y, B.x-A.x);
	if(angle<0)
		angle += 2*cn_pi;
	return angle;
}

int cn_PointPerpendLine(dbPOINT Point, LINEFORMULATION *pLine, LINEFORMULATION *pResult, dbPOINT *pPointC)
{
	double angle = cn_LineAngle(pLine);
	angle += PI/2;
	StdLineForm(Point, angle, pResult);
	cn_2LinesCrossPoint(pLine, pResult, pPointC);
	return 1;
}

int cn_PointPerpendLine(dbPOINT Point, dbPOINT A, dbPOINT B, LINEFORMULATION *pResult, dbPOINT *pPointCross)
{//过一点垂直于一条直线的方程和交点
	LINEFORMULATION line;
	StdLineForm(A, B, &line);
	return cn_PointPerpendLine(Point, &line, pResult, pPointCross);
}

int cn_PointInLineSegment(dbPOINT A, dbPOINT B, double Distance, dbPOINT *Result)
{//obtain the point which distance from A is Distance and location is in linesegment AB.
	double angle = cn_LineAngle( A, B);
	Result->x = A.x + Distance * cos(angle);
	Result->y = A.x + Distance * sin(angle);
	return 0;
}
int cn_PointDistanceInLine(dbPOINT A, dbPOINT B, double Distance, dbPOINT *Result1, dbPOINT *Result2)
{//obtain the 2 points which distance from A is Distance and location is in line AB.
	double angle = cn_LineAngle( A, B);
	Result1->x = A.x + Distance * cos(angle);
	Result1->y = A.x + Distance * sin(angle);
	Result2->x = A.x - Distance * cos(angle);
	Result2->y = A.x - Distance * sin(angle);
	return 0;
}

int cn_PointSlopeLine(dbPOINT point, double angle, LINEFORMULATION *Result)
{//点斜式直线求出标准方程
	if((angle - PI/2) == 0)
	{
		Result->a = 1;
		Result->b = 0;
		Result->c = -point.x;
	}
	else
	{
		Result->a = -tan(angle);
		Result->b = 1;
		Result->c = tan(angle)*point.x - point.y;
	}
	return 0;
}


int cn_DistanceLines(LINEFORMULATION *pRefLine, double Distance, LINEFORMULATION *Result1, LINEFORMULATION *Result2)
{//get the 2 lines that parallel the given line with the given distance
	*Result1 = *pRefLine;
	*Result2 = *pRefLine;
	if(pRefLine->a==0)
	{
		Result1->c = pRefLine->c + pRefLine->b * Distance;
		Result2->c = pRefLine->c - pRefLine->b * Distance;
		return 1;
	}
	double theta = cn_LineAngle(pRefLine);
	Result1->c = pRefLine->c + pRefLine->a * Distance / sin(theta);
	Result2->c = pRefLine->c - pRefLine->a * Distance / sin(theta);
	return 1;
}


double cn_2PointsDist(dbPOINT A, dbPOINT B)
{
	return sqrt((B.y-A.y)*(B.y-A.y) + (B.x-A.x)*(B.x-A.x));
}
double distRobot2Pt(RobotInford robot,dbPOINT point)//车到点的距离
{
	return sqrt((robot.x-point.x)*(robot.x-point.x) + (robot.y-point.y)*(robot.y-point.y));
}
double cn_AngleTrim2PI(double theta)
{////trim theta to between 0 and 2pi
/*	if(theta<-100000000000 || theta >1000000)
		theta = 0;
	while(theta>=2*PI)
		theta -= 2*PI;
	while(theta<0)
		theta += 2*PI;
	return theta;
	*/

		double cn_PI = acos(-1);
	if(theta<-100000000000 || theta >1000000)
		theta = theta;
	while(theta>=2*cn_PI)
		theta -= 2*PI;
	while(theta<0)
		theta += 2*PI;
	return theta;
}
double cn_AngleTrim2PIli(double theta)
{////trim theta to between 0 and 2PI
	double cn_PI = acos(-1);
	if(theta<-100000000000 || theta >1000000)
		theta = theta;
	while(theta>=2*cn_PI)
		theta -= 2*PI;
	while(theta<0)
		theta += 2*PI;
	return theta;
}
double cn_AngleTrimPI(double theta)
{//trim theta to the angle between 0 and pi

	theta = cn_AngleTrim2PI(theta);
	if(theta>PI)
		theta -= PI;
	return theta;
}


int cn_LineSegmentHoldin(dbPOINT P, dbPOINT A, dbPOINT B)
{//to see if P is in the segment AB, return 1 if is, otherwise
//return 0;
	double d1, d2, d3;
	if(&P==NULL || &A==NULL || &B==NULL)
		return 0;
	d1 = cn_2PointsDist(P, A);
	d2 = cn_2PointsDist(P, B);
	d3 = cn_2PointsDist(A, B);
	if(fabs((d1+d2)-cn_2PointsDist(A, B))<0.000000001)
		return 1;
	return -1;
}

void CoordinateTransform(dbROBOTPOSTURE *pNewCoordiante, dbPOINT Target, dbPOINT *Result)
{
	dbPOINT newpoint;
	newpoint.x = Target.x - pNewCoordiante->x;//平移
	newpoint.y = Target.y - pNewCoordiante->y;
	//旋转
	Result->x = newpoint.x*cos(pNewCoordiante->theta) + newpoint.y*sin(pNewCoordiante->theta);
	Result->y = -newpoint.x*sin(pNewCoordiante->theta) + newpoint.y*cos(pNewCoordiante->theta);

}

/*************** end *****************/





















/*************** end ********************/
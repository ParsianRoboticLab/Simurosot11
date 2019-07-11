#ifndef _analytic_geometry
#define _analytic_geometry
#include "Globe.h"

/********* chen ning **************/
int CalcuFormat(POINT P2, POINT P1, Formulation *Param);//aY=bX+c 

int StdLineForm(dbPOINT P1, dbPOINT P2, Formulation *FormuParam);//aX+bY+c=0
int StdLineForm(dbPOINT point, double angle, LINEFORMULATION *Result);//点斜式得出标准方程

int cn_PointSlopeLine(dbPOINT point, double angle, LINEFORMULATION *Result);//点斜式得出标准方程
int cn_2LinesCrossPoint(Formulation *Line1, Formulation *Line2, dbPOINT *Result);//两直线交点
int cn_2LinesCrossPoint(dbPOINT A1, dbPOINT A2, dbPOINT B1, dbPOINT B2, dbPOINT *Result);
double cn_LineAngle(LINEFORMULATION *pLine);//直线与x轴的夹角
double cn_LineAngle(dbPOINT A, dbPOINT B);//直线与x轴的夹角
int cn_PointPerpendLine(dbPOINT Point, LINEFORMULATION *pLine, LINEFORMULATION *pResult, dbPOINT *pPointCross);//过一点垂直于一条直线的方程和交点
int cn_PointPerpendLine(dbPOINT Point, dbPOINT A, dbPOINT B, LINEFORMULATION *pResult, dbPOINT *pPointCross);//过一点垂直于一条直线的方程和交点
int cn_DistanceLines(LINEFORMULATION *pRefLine, double Distance, LINEFORMULATION *Result1, LINEFORMULATION *Result2);//与一条直线相距一定距离的直线方程

double cn_2PointsDist(dbPOINT A, dbPOINT B);//两点间距离
double distRobot2Pt(RobotInford robot,dbPOINT point);//车到点的距离

double cn_AngleTrim2PI(double theta);//变化角度到2*pi
double cn_AngleTrimPI(double theta);//变化角度到pi

 


int cn_PointInLineSegment(dbPOINT A, dbPOINT B, double Distance, dbPOINT *Result);
int cn_PointDistanceInLine(dbPOINT A, dbPOINT B, double Distance, dbPOINT *Result1, dbPOINT *Result2);
int cn_LineSegmentHoldin(dbPOINT P, dbPOINT A, dbPOINT B);
void CoordinateTransform(dbROBOTPOSTURE *pNewCoordiante, dbPOINT Target, dbPOINT *Result);
/*********** end ************/

#endif






















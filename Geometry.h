#ifndef _analytic_geometry
#define _analytic_geometry
#include "Globe.h"

/********* chen ning **************/
int CalcuFormat(POINT P2, POINT P1, Formulation *Param);//aY=bX+c 

int StdLineForm(dbPOINT P1, dbPOINT P2, Formulation *FormuParam);//aX+bY+c=0
int StdLineForm(dbPOINT point, double angle, LINEFORMULATION *Result);//��бʽ�ó���׼����

int cn_PointSlopeLine(dbPOINT point, double angle, LINEFORMULATION *Result);//��бʽ�ó���׼����
int cn_2LinesCrossPoint(Formulation *Line1, Formulation *Line2, dbPOINT *Result);//��ֱ�߽���
int cn_2LinesCrossPoint(dbPOINT A1, dbPOINT A2, dbPOINT B1, dbPOINT B2, dbPOINT *Result);
double cn_LineAngle(LINEFORMULATION *pLine);//ֱ����x��ļн�
double cn_LineAngle(dbPOINT A, dbPOINT B);//ֱ����x��ļн�
int cn_PointPerpendLine(dbPOINT Point, LINEFORMULATION *pLine, LINEFORMULATION *pResult, dbPOINT *pPointCross);//��һ�㴹ֱ��һ��ֱ�ߵķ��̺ͽ���
int cn_PointPerpendLine(dbPOINT Point, dbPOINT A, dbPOINT B, LINEFORMULATION *pResult, dbPOINT *pPointCross);//��һ�㴹ֱ��һ��ֱ�ߵķ��̺ͽ���
int cn_DistanceLines(LINEFORMULATION *pRefLine, double Distance, LINEFORMULATION *Result1, LINEFORMULATION *Result2);//��һ��ֱ�����һ�������ֱ�߷���

double cn_2PointsDist(dbPOINT A, dbPOINT B);//��������
double distRobot2Pt(RobotInford robot,dbPOINT point);//������ľ���

double cn_AngleTrim2PI(double theta);//�仯�Ƕȵ�2*pi
double cn_AngleTrimPI(double theta);//�仯�Ƕȵ�pi

 


int cn_PointInLineSegment(dbPOINT A, dbPOINT B, double Distance, dbPOINT *Result);
int cn_PointDistanceInLine(dbPOINT A, dbPOINT B, double Distance, dbPOINT *Result1, dbPOINT *Result2);
int cn_LineSegmentHoldin(dbPOINT P, dbPOINT A, dbPOINT B);
void CoordinateTransform(dbROBOTPOSTURE *pNewCoordiante, dbPOINT Target, dbPOINT *Result);
/*********** end ************/

#endif






















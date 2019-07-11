// DecisionMakingx.cpp: implementation of the CDecisionMakingx class.
//
//////////////////////////////////////////////////////////////////////
#include "stdio.h"
#include "stdafx.h"
#include "DecisionMakingx.h"
#include "Geometry.h"
//#include "Realball.h"
//#include "BaseAgent.h"

#include "Matrix.h"
/*#ifndef realball
#include "Realball.h"
#endif
*/
//#define XIMAGE

#ifdef XIMAGE
#include "F:\Library\CxImage\CxImage\ximage.h"
#pragma comment(linker, "/libpath:\"F:\\Library\\CxImage\\lib\" ")
#pragma comment(lib, "F:\\Library\\CxImage\\lib\\cximageD.lib")
#pragma comment(lib, "F:\\Library\\CxImage\\lib\\j2kD.lib")
#pragma comment(lib, "F:\\Library\\CxImage\\lib\\jasperD.lib")
#pragma comment(lib, "F:\\Library\\CxImage\\lib\\jbigD.lib")
#pragma comment(lib, "F:\\Library\\CxImage\\lib\\jpegD.lib")
#pragma comment(lib, "F:\\Library\\CxImage\\lib\\pngD.lib")
#pragma comment(lib, "F:\\Library\\CxImage\\lib\\tiffD.lib")
#pragma comment(lib, "F:\\Library\\CxImage\\lib\\zlib.lib")
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//define TEST1

void CDecisionMakingx::Test() {
//	HundunTest();
//	return;
	
	VecPosition pos(220, 90);
//	ToPositionN(&Robot[2], m_posBall, 110, &rbV[2]);

//	ToPositionPD(&Robot[2], m_posBall, 70, &rbV[2]);
//	TurnToAnglePD(&Robot[2], -2*PI/4, ANTICLOCK, &rbV[2]);
//	TurnToPointPD(&Robot[2], m_posBall, NOCLOCK, &rbV[2]);
//	MoveOnAngle(&Robot[2], -PI/4, 70, &rbV[2]);
//	MoveToPt(&Robot[2], m_posBall, 70, &rbV[2]);
//	ToPositionPDGoal(&Robot[4], m_posBall, 40, 0, &rbV[4]);
//	ToPositionNew(&Robot[2], pos, m_posBall, 110, 2, &rbV[2]);

//	if (m_nTimer < 10)
	rbV[2].LeftValue = rbV[2].RightValue = -110;
//	ShootLine(2);
//	Vect_MidShoot(2);

//	GoalieAction(4);
//	ToPositionPDGoal(&Robot[4], pos, 110, 0, &rbV[4]);
//	for (int i=0; i<11; i++)
	ToPositionN(3, m_posBall, 110);
//	ToPositionPDGoal(4, pos, 110, 0);
//	Wait(0, pos, 0, 110);
//	BoundPushBall(&Robot[3], &rbV[3]);

//	Robot[4].x = 5.54; Robot[4].y = 76.69; Robot[4].theta = -1.57;
//	pos.SetX(5.50); pos.SetY(72.00);
//	ToPositionPDGoal(&Robot[4], pos, MAXSPEED, 50, &rbV[4]);

//	Kick2(2);
}

void CDecisionMakingx::HundunTest() {
	VecPosition q[10];
	
	q[0] = m_posBall;
	q[1].SetVecPosition(Robot[2].x, Robot[2].y);
	q[2].SetVecPosition(Robot[0].x, Robot[0].y);
	q[3].SetVecPosition(Robot[3].x, Robot[3].y);
	q[4].SetVecPosition(Robot[4].x, Robot[4].y);
	
	ToPositionAvoidObstacles(1, q, 4);

//	LogData();//��

//	if (m_nTimer % 10 == 1)
//		Capture();
}

CDecisionMakingx::CDecisionMakingx() :
		nettpt("Data\\tpt.net"), nettpn("Data\\tpn.net") {
	m_formation.ReadFormation("Data\\Strategy.nse");

//	m_svmModelTpt = svm_load_model("Data\\tpt.model");
//	m_svmAttrTpt = (struct svm_node *) malloc(4*sizeof(struct svm_node));
	
	m_timerStart.AttachListener(this);
	m_timerShootLine.AttachListener(this);
	m_timerBallKick.AttachListener(this);
	
	Initialize();//��ʼ������
}

CDecisionMakingx::~CDecisionMakingx() {
//	svm_destroy_model(m_svmModelTpt);
//	free(m_svmAttrTpt);
}

/////////////// Basic Actions ///////////////////////////////////////////////////////
/************************************************************************/
/* ���ϵ�����                                                           */
/************************************************************************/
void CDecisionMakingx::ToPositionAvoidObstacles(int nRobot, VecPosition pos[],
                                                int nObstacles) {
	dbLRWheelVelocity *pSpeed = &rbV[nRobot];
	
	// Hundun�㷨�������,q[0]=��ǰ�㣬q[1]=Ŀ��㣬�����ϰ����
	VecPosition *q = new VecPosition[nObstacles + 2];
	
	q[0].SetVecPosition(Robot[nRobot].x, Robot[nRobot].y);
	q[1] = pos[0];
	
	for (int i = 0; i < nObstacles; i++)
		q[i + 2] = pos[i + 1];
	
	double theta = Robot[nRobot].theta;
	
	// ת������ϵ��Hundun�㷨ĿǰҪ��Ŀ�������ʼ������
	BOOL bRight = TRUE, bTop = TRUE;
	// ����ת��
	if (q[1].GetX() < q[0].GetX()) {
		bRight = FALSE;
		for (int i = 0; i < nObstacles + 2; i++)
			q[i].SetX(PITCH_LENGTH - q[i].GetX());
		
		theta = VecPosition::NormalizeAngle(PI - theta);
	}
	// ����ת��
	if (q[1].GetY() < q[0].GetY()) {
		bTop = FALSE;
		for (int i = 0; i < nObstacles + 2; i++)
			q[i].SetY(PITCH_WIDTH - q[i].GetY());
		
		theta = VecPosition::NormalizeAngle(-theta);
	}
	
	m_hundun.hundun(q, nObstacles);
	
	double dTheta = m_hundun.X[1] - theta;
	
	double w = dTheta;
	double v = (7 - nObstacles) * m_hundun.X[0];
	double L = 4;
	double VL = v - 10 * w * L;
	double VR = v + 10 * w * L;
	
	pSpeed->LeftValue = VL;
	pSpeed->RightValue = VR;
	
	if (!bRight) {
		double temp = pSpeed->LeftValue;
		pSpeed->LeftValue = pSpeed->RightValue;
		pSpeed->RightValue = temp;
	}
	if (!bTop) {
		double temp = pSpeed->LeftValue;
		pSpeed->LeftValue = pSpeed->RightValue;
		pSpeed->RightValue = temp;
	}

//	LimitSpeed(&rbV[1], 110, TRUE);
	
	if (sqrt((m_hundun.XYTO[0] - m_hundun.xg) * (m_hundun.XYTO[0] - m_hundun.xg)
	         + (m_hundun.XYTO[1] - m_hundun.yg) * (m_hundun.XYTO[1] - m_hundun.yg))
	    < 3) {
		pSpeed->LeftValue = 0;
		pSpeed->RightValue = 0;
	}
	
	delete[] q;
}


/************************************************************************/
/* ����������ʹС���ķ����Ը�����ʱ�ӷ������ת����Ҫ��ĽǶȷ���		*/
/*  �ɵ����� ��Kp��KdΪ������΢�ֵ���, angle�Ƕ����					*/
/************************************************************************/
void CDecisionMakingx::TurnToAnglePD(int nRobot, double dAngle, int clock) {
	dbLRWheelVelocity *pSpeed = &rbV[nRobot];
	
	static double anglePrev[ROBOTNUMBER] = {0};
	
	double angle, dSameSpeed;
	int nQuadrant = 0;
	angle = Robot[nRobot].theta - dAngle;
	angle = VecPosition::NormalizeAngle(angle);
	if (angle < 0)
		angle += 2 * PI;
	
	if (clock == ANTICLOCK) {
		angle = 2 * PI - angle;
	} else if (clock == NOCLOCK) {
		angle = VecPosition::NormalizeAngle(angle);
		
		//�жϽǶȲ���������
		if (angle >= -PI && angle < -PI / 2) {
			nQuadrant = 3;
			angle = angle + PI;
		} else if (angle >= -PI / 2 && angle < 0) {
			nQuadrant = 4;
			angle = -angle;
		} else if (angle >= 0 && angle < PI / 2) {
			nQuadrant = 1;
		} else if (angle >= PI / 2 && angle <= PI) {
			nQuadrant = 2;
			angle = PI - angle;
		}
	}

#ifdef SimuroSot5
																															double angleMax = 2.0/180.0*PI;
	//�˴�����PD����
	double Kp = 10;
	double Kd = 50;
#endif

#ifdef SimuroSot11
	double angleMax = 2.0 / 180.0 * PI;
	double Kp = 10;
	double Kd = 50;
#endif
	
	if (fabs(angle) <= angleMax)//�ж��Ƿ��ڽǶ������֮��
	{
		pSpeed->LeftValue = 0;
		pSpeed->RightValue = 0;
		return;
	}
	
	dSameSpeed = Kp * angle + Kd * (angle - anglePrev[nRobot]);
	dSameSpeed = Maths::Limit(dSameSpeed, 0, 40);
	
	anglePrev[nRobot] = angle;
	
	if (clock == CLOCKWISE) {
		pSpeed->LeftValue = dSameSpeed;
		pSpeed->RightValue = -dSameSpeed;
	} else if (clock == ANTICLOCK) {
		pSpeed->LeftValue = -dSameSpeed;
		pSpeed->RightValue = dSameSpeed;
	} else {
		switch (nQuadrant) {
			case 1://˳ʱ����ת
			case 3: {
				pSpeed->LeftValue = dSameSpeed;
				pSpeed->RightValue = -dSameSpeed;
				break;
			}
			case 2://��ʱ����ת
			case 4: {
				pSpeed->LeftValue = -dSameSpeed;
				pSpeed->RightValue = dSameSpeed;
				break;
			}
		}
	}
}

/************************************************************************/
/*	����������ʹС������ת��ָ����										*/
/*	RobotΪС��λ����Ϣ��PointΪת��ĵ㣬pSpeedΪ���ص���������		*/
/*	Kp��KdΪ������΢�ֵ��ڲ���											*/
/************************************************************************/
void CDecisionMakingx::TurnToPointPD(int nRobot, VecPosition posTarget, int clock) {
	double dAngle = (posTarget - VecPosition(Robot[nRobot].x, Robot[nRobot].y)).GetDirection();
	TurnToAnglePD(nRobot, dAngle, clock);
}

/************************************************************************/
/*	���������ڶ�����ת���ɹ�����1��										*/
/*	VelocityΪҪ����ת�Ľ��ٶȣ�VelocityΪ��������ʱ�뷽����ת��Ϊ����	*/
/*  ��˳ʱ�뷽����ת��pSpeedΪ���ص���������							*/
/************************************************************************/
void CDecisionMakingx::Turn(int nRobot, double dVelocity, int clock) {
	dbLRWheelVelocity *pSpeed = &rbV[nRobot];
	
	if (clock == CLOCKWISE) {
		pSpeed->LeftValue = dVelocity;
		pSpeed->RightValue = -dVelocity;
	} else {
		pSpeed->LeftValue = -dVelocity;
		pSpeed->RightValue = dVelocity;
	}
}

/************************************************************************/
/*  ����������С����ĳһ�Ƕȷ����ƶ�,                                   */
/*  ���������㷨
/************************************************************************/
void CDecisionMakingx::MoveOnAngle(int nRobot, double Angle, double speed) {
	dbLRWheelVelocity *pSpeed = &rbV[nRobot];
	
	double dDifference;
	dDifference = Robot[nRobot].theta - Angle;
	dDifference = VecPosition::NormalizeAngle(dDifference);
	
	if (dDifference >= -PI && dDifference < -PI / 2) {
		pSpeed->RightValue = -speed;
		pSpeed->LeftValue = pSpeed->RightValue * cos(dDifference + PI);
	} else if (dDifference >= -PI / 2 && dDifference < 0) {
		pSpeed->RightValue = speed;
		pSpeed->LeftValue = pSpeed->RightValue * cos(-dDifference);
	} else if (dDifference >= 0 && dDifference < PI / 2) {
		pSpeed->LeftValue = speed;
		pSpeed->RightValue = pSpeed->LeftValue * cos(dDifference);
	} else if (dDifference >= PI / 2 && dDifference < PI) {
		pSpeed->LeftValue = -speed;
		pSpeed->RightValue = pSpeed->LeftValue * cos(PI - dDifference);
	}
}

/************************************************************************/
/* �����������Ұ��������ٶȵ�����                                       */
/* �ɵ�������Kp, ����ϵ��
/************************************************************************/
void CDecisionMakingx::MoveToPt(int nRobot, VecPosition posTarget, double speed) {
	dbLRWheelVelocity *pSpeed = &rbV[nRobot];
	
	double theta, temp_theta;
	double dx, dy, dx1, dy1;
	double distance;
	
	//����任����ƽ�ƺ�ת�ǣ���С������Ϊԭ�㣬С������Ϊy��
	dx1 = posTarget.GetX() - Robot[nRobot].x;
	dy1 = posTarget.GetY() - Robot[nRobot].y;
	dx = dx1 * cos(Robot[nRobot].theta - PI / 2) + dy1 * sin(Robot[nRobot].theta - PI / 2);
	dy = -dx1 * sin(Robot[nRobot].theta - PI / 2) + dy1 * cos(Robot[nRobot].theta - PI / 2);
	
	distance = sqrt(dx * dx + dy * dy);
	theta = atan2(dy, dx);
	
	double Kp = 0.8;
	
	if (theta >= -PI && theta < -PI / 2) {
		temp_theta = -PI / 2 - theta;
		if (fabs(temp_theta) < PI / 7) temp_theta *= 2;
		pSpeed->LeftValue = -cos(temp_theta) * Kp * speed;
		pSpeed->RightValue = -speed;
	} else if (theta >= -PI / 2 && theta < 0) {
		temp_theta = theta + PI / 2;
		if (fabs(temp_theta) < PI / 7) temp_theta *= 2;
		pSpeed->LeftValue = -speed;
		pSpeed->RightValue = -cos(temp_theta) * Kp * speed;
	} else if (theta >= 0 && theta < PI / 2)//��һ����
	{
		temp_theta = PI / 2 - theta;
		if (temp_theta < PI / 7) temp_theta *= 2;
		pSpeed->LeftValue = speed;
		pSpeed->RightValue = cos(temp_theta) * Kp * speed;
	} else if (theta >= PI / 2 && theta < PI)//�ڶ�����
	{
		temp_theta = theta - PI / 2;
		if (fabs(temp_theta) < PI / 7) temp_theta *= 2;
		pSpeed->LeftValue = cos(temp_theta) * Kp * speed;
		pSpeed->RightValue = speed;
	}
}

/************************************************************************/
/*  ������,PD�㷨                                                       */
/*  �ɵ�������Kp,Kd:���ٶ�w��PDϵ��	;angle:���Ƕ�ƫ�����angleʱ����ת	*/
/************************************************************************/
void CDecisionMakingx::ToPositionPD(int nRobot, VecPosition posTarget,
                                    double speed) {
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	dbLRWheelVelocity *pSpeed = &rbV[nRobot];
	
	//����Ƕ�ƫ��
	static double distPrev[ROBOTNUMBER] = {0};
	static double anglePrev[ROBOTNUMBER] = {0};
	
	double dist = (posRobot - posTarget).GetMagnitude();
	double angle = VecPosition::NormalizeAngle(
			(posTarget - posRobot).GetDirection() - Robot[nRobot].theta);


#ifdef SimuroSot5
																															double angleMax = PI*0.48;	// ���ڴ˽Ƕ���ƫת
	double kpa = 100, kda = 0;
	double kp = 50, kd = 300;
#endif

#ifdef SimuroSot11
	double angleMax = PI * 0.48;    // ���ڴ˽Ƕ���ƫת
	double kpa = 100, kda = 0;
	double kp = 50, kd = 300;
#endif
	
	
	double speed_e;
	
	int nQuadrand = -1;
	if (angle > -PI && angle < -PI / 2) {
		nQuadrand = 3;
		angle = angle + PI;
	} else if (angle > -PI / 2 && angle < 0) {
		nQuadrand = 4;
		angle = -angle;
	} else if (angle > 0 && angle <= PI / 2) {
		nQuadrand = 1;
		angle = angle;
	} else if (angle > PI / 2 && angle <= PI) {
		nQuadrand = 2;
		angle = PI - angle;
	}
	if (dist < 0.2)
		dist = 0;
	if (angle < PI / 90)
		angle = 0;
	
	if (angle < angleMax) {
		speed_e = kpa * angle + kda * (angle - anglePrev[nRobot]);
		speed = kp * dist + kd * (dist - distPrev[nRobot]); // speed*(angleMax-angle)/angleMax;
	} else {
		speed_e = kpa * angle + kda * (angle - anglePrev[nRobot]);
		speed = 0;
	}
	distPrev[nRobot] = dist;
	anglePrev[nRobot] = angle;
	
	switch (nQuadrand) {
		case 1:
			pSpeed->LeftValue = speed - speed_e / 2;
			pSpeed->RightValue = speed + speed_e / 2;
			break;
		case 2:
			pSpeed->LeftValue = -speed + speed_e / 2;
			pSpeed->RightValue = -speed - speed_e / 2;
			break;
		case 3:
			pSpeed->LeftValue = -speed - speed_e / 2;
			pSpeed->RightValue = -speed + speed_e / 2;
			break;
		case 4:
			pSpeed->LeftValue = speed + speed_e / 2;
			pSpeed->RightValue = speed - speed_e / 2;
			break;
	}
	
	
	LimitSpeed(pSpeed, MAXSPEED, FALSE);

/*	// ����
	double distBuffer = Robot[nRobot].speed / 30;
	if (dist < distBuffer)
	{
		LimitSpeed(pSpeed, 0, 0);
	}
*/
}


/************************************************************************/
/* �����㺯��                                                           */
/* �ɵ�������kp4new : ���ٶ�Kϵ����angle:���Ƕ�ƫ�����angleʱ����ת	*/
/************************************************************************/
void CDecisionMakingx::ToPositionN(int nRobot, VecPosition posTarget, double speed) {
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	dbLRWheelVelocity *pSpeed = &rbV[nRobot];
	
	//����Ƕ�ƫ��
	static double distPrev[ROBOTNUMBER] = {0};
	static double anglePrev[ROBOTNUMBER] = {0};
	
	double dist = (posRobot - posTarget).GetMagnitude();
	double angle = VecPosition::NormalizeAngle(
			(posTarget - posRobot).GetDirection() - Robot[nRobot].theta);


#ifdef SimuroSot5
																															double angleMax = PI*0.48;	// ���ڴ˽Ƕ���ƫת
	double kpa = 100, kda = 0;
	double kp = 50, kd = 300;
#endif

#ifdef SimuroSot11  //�޸� :�����S 2004.6.11
	double angleMax = PI * 0.48;    // ���ڴ˽Ƕ���ƫת
	double kpa = 100, kda = 0;
	double kp = 50, kd = 300;
#endif
	
	double speed_e;
	
	int nQuadrand = -1;
	if (angle > -PI && angle < -PI / 2) {
		nQuadrand = 3;
		angle = angle + PI;
	} else if (angle > -PI / 2 && angle < 0) {
		nQuadrand = 4;
		angle = -angle;
	} else if (angle > 0 && angle <= PI / 2) {
		nQuadrand = 1;
		angle = angle;
	} else if (angle > PI / 2 && angle <= PI) {
		nQuadrand = 2;
		angle = PI - angle;
	}
	if (dist < 0.2)
		dist = 0;
	if (angle < PI / 90)
		angle = 0;
	
	{
		SendMsg(0, "%5.5f %5.5f %5.5f %5.5f",
		        dist, angle, Robot[nRobot].speedv, Robot[nRobot].speedw);
	}
	
	if (angle < angleMax) {
		speed_e = kpa * angle + kda * (angle - anglePrev[nRobot]);
		speed = speed * (angleMax - angle) / angleMax;
	} else {
		speed_e = kpa * angle + kda * (angle - anglePrev[nRobot]);
		speed = 0;
	}
	distPrev[nRobot] = dist;
	anglePrev[nRobot] = angle;
	
	switch (nQuadrand) {
		case 1:
			pSpeed->LeftValue = speed - speed_e / 2;
			pSpeed->RightValue = speed + speed_e / 2;
			break;
		case 2:
			pSpeed->LeftValue = -speed + speed_e / 2;
			pSpeed->RightValue = -speed - speed_e / 2;
			break;
		case 3:
			pSpeed->LeftValue = -speed - speed_e / 2;
			pSpeed->RightValue = -speed + speed_e / 2;
			break;
		case 4:
			pSpeed->LeftValue = speed + speed_e / 2;
			pSpeed->RightValue = speed - speed_e / 2;
			break;
	}
	
	
	LimitSpeed(pSpeed, MAXSPEED, TRUE);

/*	// ����
	double distBuffer = Robot[nRobot].speed / 30;
	if (dist < distBuffer)
	{
		LimitSpeed(pSpeed, 0, 0);
	}
*/
}

/************************************************************************/
/*  ����Ա�õ����㺯��                                                  */
/************************************************************************/
void CDecisionMakingx::ToPositionPDGoal(int nRobot, VecPosition posTarget,
                                        double speed, double endspeed) {
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	dbLRWheelVelocity *pSpeed = &rbV[nRobot];
	
	// ����ܽ���ʱ�򣬲�Ҫ�����Ƕ�
	if (posRobot.GetDistanceTo(m_posBall) < 20)
		posTarget.SetX(posRobot.GetX());
	
	static double distPrev[ROBOTNUMBER] = {0};
	static double anglePrev[ROBOTNUMBER] = {0};
	
	double dist = (posRobot - posTarget).GetMagnitude();
	double angle = VecPosition::NormalizeAngle(
			(posTarget - posRobot).GetDirection() - Robot[nRobot].theta);
	
	if (dist < 1)
		angle = VecPosition::NormalizeAngle(
				PI / 2 - Robot[nRobot].theta);
	
	
	double kp4new = 100;
	
	if (dist > 100)
		kp4new = 15;
	else if (dist > 50)
		kp4new = 20;
	else if (dist > 30)
		kp4new = 25;
	else if (dist > 20)
		kp4new = 30;
	else
		kp4new = 100;
	
	if (dist < 5.0 && fabs(angle) < 40)
		kp4new = 5;


/*	double dBufferDist = 50;
	if (dist < dBufferDist)
		speed =  endspeed + dist * (speed - endspeed)/dBufferDist;

	if (speed < endspeed)
		speed = endspeed;
*/

#ifdef SimuroSot5
																															double angleMax = PI*89./180.;	// ���ڴ˽Ƕ���ƫת
	double kp = 45, ki=0, kd=250;
	double kpa = 100, kia = 0, kda = 100;
#endif

#ifdef SimuroSot11
	double angleMax = PI * 89. / 180.;    // ���ڴ˽Ƕ���ƫת
	double kp = 45, ki = 0, kd = 250;
	double kpa = 100, kia = 0, kda = 100;
#endif
	
	double speed_e;
	
	int nQuadrand = -1;
	if (angle > -PI && angle < -PI / 2) {
		nQuadrand = 3;
		angle = angle + PI;
	} else if (angle > -PI / 2 && angle < 0) {
		nQuadrand = 4;
		angle = -angle;
	} else if (angle > 0 && angle <= PI / 2) {
		nQuadrand = 1;
		angle = angle;
	} else if (angle > PI / 2 && angle <= PI) {
		nQuadrand = 2;
		angle = PI - angle;
	}
	
	if (dist < 0.2)
		dist = 0;
	if (angle < PI / 90)
		angle = 0;
	
	
	static double distSum = 0;
/*	if (posTarget.GetY() > Robot[nRobot].y)
		dist = -dist;
	distSum += dist;
*/
	if (angle < angleMax) {
		speed_e = kpa * angle + kda * (angle - anglePrev[nRobot]);
		speed = kp * dist + ki * distSum +
		        kd * (dist - distPrev[nRobot]); //speed*(1.0 / (1.0 + exp(-3.0 * dist)) - 0.3);
	} else {
		speed_e = kpa * angle + kda * (angle - anglePrev[nRobot]);
		speed = 0;
	}
	distPrev[nRobot] = dist;
	anglePrev[nRobot] = angle;
	
	SendMsg(0, "%2.2f, %2.2f %d %2.2f %2.2f", dist, angle, nQuadrand, speed, speed_e);
	
	switch (nQuadrand) {
		case 1:
			pSpeed->LeftValue = speed - speed_e / 2;
			pSpeed->RightValue = speed + speed_e / 2;
			break;
		case 2:
			pSpeed->LeftValue = -speed + speed_e / 2;
			pSpeed->RightValue = -speed - speed_e / 2;
			break;
		case 3:
			pSpeed->LeftValue = -speed - speed_e / 2;
			pSpeed->RightValue = -speed + speed_e / 2;
			break;
		case 4:
			pSpeed->LeftValue = speed + speed_e / 2;
			pSpeed->RightValue = speed - speed_e / 2;
			break;
	}
	
	LimitSpeed(pSpeed, MAXSPEED, FALSE);
	
	// ����
/*	double distBuffer = Robot[ROBOTNUMBER].speed / 30;
	if (dist < distBuffer)
	{
		LimitSpeed(pSpeed, 0, FALSE);
	}
*/
	
	SendMsg(0, "%2.2f, %2.2f", pSpeed->LeftValue, pSpeed->RightValue);
}

/************************************************************************/
/* ���򵽶��㺯��                                                       */
/* �ɵ�������kp4new, P������angle��ת���Ƕ�								*/
/* angleLimit1,2; distLimit1,2											*/
/************************************************************************/
void CDecisionMakingx::ToPositionNew(int nRobot, VecPosition posTarget, VecPosition posBall,
                                     double speed, int IfEndprocess) {
	dbLRWheelVelocity *pSpeed = &rbV[nRobot];

//	VecPosition posBall = m_posBall;
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	
	double distBall2Robot = (posBall - posRobot).GetMagnitude();
	double angleBall2Robot = (posBall - posRobot).GetDirection();
	double angleTarget2Ball = (posTarget - posBall).GetDirection();
	
	
	{
		SendMsg(0, "%5.5f %5.5f %5.5f %5.5f",
		        distBall2Robot, angleBall2Robot - Robot[nRobot].theta,
		        angleTarget2Ball - angleBall2Robot, Robot[nRobot].speedv);
	}
	
	double angleE = angleTarget2Ball - angleBall2Robot;
	
	int sign = Maths::Sign(angleE);

/************************************************************************/
/* ToPositionNew������Ŀ��ǶȲ�������                                  */
/* �ɵ�������angleLimit1,angleLimit2, distLimit1, distLimit2			*/
/************************************************************************/
	double anglelimit1 = PI * 0.5;
	double anglelimit2 = PI * .12;
	double distlimit1 = 200;
	double distlimit2 = 10;
	
	double ktheta = (distlimit1 - distBall2Robot) / (distlimit1 - distlimit2);
	ktheta = Maths::Limit(ktheta, 0, 1);
	
	double tempp = 0;
	if (distBall2Robot <= distlimit1) {
		tempp = (fabs(angleE) - anglelimit2) / (anglelimit1 - anglelimit2);
		tempp = Maths::Limit(tempp, 0, 1);
		tempp *= PI / 2;
		
		tempp = ktheta * tempp;
	}
/*	double l1,l3;
	l1 = 20;
	l3=1-(l1-distBall2Robot-fabs(angleE)*150)/l1;
	l3 = Maths::Limit(l3, 0, 1);
	tempp *= l3;
*/
	///< ��distRobot2Ball > distLimit2 || angleE < angleLimit2ʱ����ֱ������
	///< ��distRobot2Ball < distLimit1 && angleE > angleLimit1ʱ�����ת��PI/2
	
	//�����¸����ڵ�Ŀ��Ƕ�
	double disiredAngle = angleBall2Robot - sign * tempp;
	
	//����Ƕ�ƫ��
	double Angle_e;
	Angle_e = disiredAngle - Robot[nRobot].theta;
	Angle_e = VecPosition::NormalizeAngle(Angle_e);
	
	
	//�����������ٶȲ������������ٶ�
	double angle = PI * .5;
//	if(Robot[nRobot].x<7||Robot[nRobot].x>PITCH_LENGTH-7||Robot[nRobot].y<7||Robot[nRobot].y>PITCH_WIDTH-7)
//		angle = PI*.27;
	
	double speed_e;
	
	double kp4new = 80;
	
	double distLimit3 = 35;
	double distLimit4 = 15;
	double kp4Max = 90;
	double kp4Max2 = 30;
	
	if (Angle_e > -PI && Angle_e < -PI / 2) {
		if (distBall2Robot < distLimit3 && distBall2Robot > distLimit4)
			kp4new = kp4new * (Angle_e + PI) / (PI / 2) * (distLimit3 / distBall2Robot);
		else if (distBall2Robot < distLimit4)
			kp4new = kp4Max2;
		
		kp4new = Maths::Limit(kp4new, 0, kp4Max);
		
		speed_e = kp4new * (Angle_e + PI);
		if (fabs(Angle_e + PI) > angle)
			speed = 0;
		else
			speed = speed * (PI / 2 - fabs(Angle_e + PI)) / angle;
		
		pSpeed->LeftValue = speed + speed_e / 2;
		pSpeed->RightValue = speed - speed_e / 2;
		pSpeed->LeftValue = -pSpeed->LeftValue;
		pSpeed->RightValue = -pSpeed->RightValue;
	} else if (Angle_e > -PI / 2 && Angle_e < 0) {
		if (distBall2Robot < distLimit3 && distBall2Robot > distLimit4)
			kp4new = kp4new * (Angle_e + PI) / (PI / 2) * (distLimit3 / distBall2Robot);
		else if (distBall2Robot < distLimit4)
			kp4new = kp4Max2;
		
		kp4new = Maths::Limit(kp4new, 0, kp4Max);
		
		speed_e = kp4new * (-Angle_e);
		if (fabs(-Angle_e) > angle)
			speed = 0;
		else
			speed = speed * (PI / 2 - fabs(-Angle_e)) / angle;
		
		pSpeed->LeftValue = speed + speed_e / 2;
		pSpeed->RightValue = speed - speed_e / 2;
	} else if (Angle_e >= 0 && Angle_e < PI / 2)//�Ƕ�ƫ���ڵ�һ����
	{
		if (distBall2Robot < distLimit3 && distBall2Robot > distLimit4)
			kp4new = kp4new * (Angle_e + PI) / (PI / 2) * (distLimit3 / distBall2Robot);
		else if (distBall2Robot < distLimit4)
			kp4new = kp4Max2;
		
		kp4new = Maths::Limit(kp4new, 0, kp4Max);
		
		speed_e = kp4new * Angle_e;
		if (fabs(Angle_e) > angle)
			speed = 0;
		else
			speed = speed * (PI / 2 - fabs(Angle_e)) / angle;
		
		pSpeed->LeftValue = speed - speed_e / 2;
		pSpeed->RightValue = speed + speed_e / 2;
	} else if (Angle_e >= PI / 2 && Angle_e <= PI) {
		if (distBall2Robot < distLimit3 && distBall2Robot > distLimit4)
			kp4new = kp4new * (Angle_e + PI) / (PI / 2) * (distLimit3 / distBall2Robot);
		else if (distBall2Robot < distLimit4)
			kp4new = kp4Max2;
		
		kp4new = Maths::Limit(kp4new, 0, kp4Max);
		
		speed_e = kp4new * (PI - Angle_e);
		if (fabs(PI - Angle_e) > angle)
			speed = 0;
		else
			speed = speed * (PI / 2 - fabs(PI - Angle_e)) / angle;
		
		pSpeed->LeftValue = speed - speed_e / 2;
		pSpeed->RightValue = speed + speed_e / 2;
		pSpeed->LeftValue = -pSpeed->LeftValue;
		pSpeed->RightValue = -pSpeed->RightValue;
	}

//	SendMsg(m_nTimer, "angleE=%2.2f, tempp = %2.2f, kp4 = %2.2f, speede=%2.2f",
//		angleE, tempp, kp4new, speed_e);
	
	
	EndProcess(IfEndprocess, nRobot, posTarget, posBall);
	
	LimitSpeed(pSpeed, MAXSPEED);
}



/************************************************************************/
/* ĩ�˴���(����Ӧ�õ����ڳ�������)                                   */
/************************************************************************/
void CDecisionMakingx::EndProcess(int IfEndprocess, int nRobot,
                                  VecPosition posTarget, VecPosition posBall) {
	dbLRWheelVelocity *pSpeed = &rbV[nRobot];
	
	dbPOINT shoot_target;
	shoot_target.x = posTarget.GetX();
	shoot_target.y = posTarget.GetY();
	dbPOINT ballPt;
	ballPt.x = posBall.GetX();
	ballPt.y = posBall.GetY();


/************************************************************************/
/*  ĩ��Բ��                                                            */
/*	�ɵ�����: r, samespeed												*/
/************************************************************************/
/////////    ///////////////////////////////////////////////////////////////////////
	double dist, distE, anglerb2ball, anglerb2target;
	double angle1, angle2, angle3, angle4, angleball2target, angle5;
	dbPOINT rbPt, EGoal;
	
	EGoal.x = PITCH_LENGTH + 3;
	EGoal.y = PITCH_WIDTH / 2;
	
	dist = distRobot2Pt(Robot[nRobot], ballPt);//������С���ľ���
	rbPt.x = Robot[nRobot].x;
	rbPt.y = Robot[nRobot].y;
	anglerb2ball = Getpt2ptAngle(rbPt, ballPt);//С��ָ����ķ���
	anglerb2target = Getpt2ptAngle(rbPt, EGoal);//С��ָ��Ŀ��ķ���
	angleball2target = Getpt2ptAngle(ballPt, shoot_target);//��Ŀ�귽��
	angle1 = VecPosition::NormalizeAngle2PI(anglerb2ball - anglerb2target);
	
	angle1 = VecPosition::NormalizeAngle(angle1);
	
	angle2 = VecPosition::NormalizeAngle2PI(Robot[nRobot].theta);
	angle3 = VecPosition::NormalizeAngle2PI(angle2 - anglerb2ball);
	
	// r��Բ�뾶��С
	double r = 18;//2.65;
	double radiu;
	
	LINEFORMULATION line1, line2, line3;
	dbPOINT tempPt1, tempPt2, tempPt3;
	tempPt1.x = Robot[nRobot].x;
	tempPt1.y = Robot[nRobot].y;
	StdLineForm(tempPt1, Robot[nRobot].theta, &line1);//��бʽ�ó�����ֱ�߷���L1
	cn_PointPerpendLine(tempPt1, &line1, &line2, &tempPt2);//������L1�Ĵ��߷���L2
	StdLineForm(tempPt1, ballPt, &line1);//�������߷���L3
	tempPt2.x = (Robot[nRobot].x + ballPt.x) / 2;
	tempPt2.y = (Robot[nRobot].y + ballPt.y) / 2;//�����е�c
	cn_PointPerpendLine(tempPt2, &line1, &line3, &tempPt3);//��c��L3�Ĵ��߷���
	cn_2LinesCrossPoint(&line2, &line3, &tempPt3);//L2��L3�Ľ��㼴Բ��
	
	double angle8;
	dbPOINT roundpt;
	roundpt = tempPt3;
	angle8 = VecPosition::NormalizeAngle2PI(angle2 - Getpt2ptAngle(rbPt, ballPt));
	
	radiu = cn_2PointsDist(tempPt3, ballPt);
	StdLineForm(tempPt3, ballPt, &line1);//Բ�������߷���L4
	cn_PointPerpendLine(ballPt, &line1, &line2, &tempPt2);//������L4�Ĵ��߷���L5,L5��Ϊ����ʱ������
	
	
	angle4 = angle2 - anglerb2ball;
	
	if (rbPt.y < -(line2.a * rbPt.x + line2.c) / line2.b) {
		if (angle8 <= PI) {
			angle5 = VecPosition::NormalizeAngle2PI(anglerb2ball - angle4);
		} else {
			angle5 = VecPosition::NormalizeAngle2PI(anglerb2ball + PI - angle4);
		}
	} else {
		if (angle8 <= PI) {
			angle5 = VecPosition::NormalizeAngle2PI(anglerb2ball + PI - angle4);
		} else {
			angle5 = VecPosition::NormalizeAngle2PI(anglerb2ball - angle4);
		}
	}
	
	if (angle5 > PI)
		angle5 -= 2 * PI;
	dbPOINT pt1, pt2;
	pt1.x = PITCH_LENGTH;
	pt1.y = PITCH_WIDTH / 2 + GOAL_WIDTH / 2;
	pt2.x = PITCH_LENGTH;
	pt2.y = PITCH_WIDTH / 2 - GOAL_WIDTH / 2;
	double angle6, angle7;
	angle6 = Getpt2ptAngle(ballPt, pt1);
	if (angle6 > PI)
		angle6 -= 2 * PI;
	angle7 = Getpt2ptAngle(ballPt, pt2);
	if (angle7 > PI)
		angle7 -= 2 * PI;
	
	//�ǶȲ���һ����Χʱ��Բ�켣ǰ��
	if (angle5 > angle7
	    && angle5 < angle6
	    && angle2 != anglerb2ball
	    && line2.b != 0
	    && rbPt.x < ballPt.x
	    && ballPt.x < EGoal.x
	    && IfEndprocess > 2) {
		double samespeed = radiu / 150 * 40 + 80;
		if (samespeed > MAXSPEED)
			samespeed = MAXSPEED;

//		SendMsg(2, "E3 : %2.2f, %2.2f", radiu, samespeed);
		
		double angleround;
		angleround = VecPosition::NormalizeAngle2PI(Getpt2ptAngle(roundpt, rbPt) - Getpt2ptAngle(roundpt, ballPt));
		
		if (rbPt.y < -(line2.a * rbPt.x + line2.c) / line2.b && angleround < PI * 1.2) {
			if (angle8 <= PI) {
				pSpeed->LeftValue = (radiu + r) / radiu * samespeed;
				pSpeed->RightValue = (radiu - r) / radiu * samespeed;
				if (pSpeed->LeftValue > MAXSPEED) {
					pSpeed->RightValue = MAXSPEED - pSpeed->LeftValue + pSpeed->RightValue;
					pSpeed->LeftValue = MAXSPEED;
				}
			} else {
				pSpeed->RightValue = -(radiu + r) / radiu * samespeed;
				pSpeed->LeftValue = -(radiu - r) / radiu * samespeed;
				if (pSpeed->RightValue < -MAXSPEED) {
					pSpeed->LeftValue = -MAXSPEED + pSpeed->RightValue - pSpeed->LeftValue;
					pSpeed->RightValue = -MAXSPEED;
				}
			}
		} else if (rbPt.y >= -(line2.a * rbPt.x + line2.c) / line2.b && (2 * PI - angleround) < PI * 1.2)//xue 4.26
		{
			if (angle8 <= PI) {
				pSpeed->LeftValue = -(radiu + r) / radiu * samespeed;
				pSpeed->RightValue = -(radiu - r) / radiu * samespeed;
				if (pSpeed->LeftValue < -MAXSPEED) {
					pSpeed->RightValue = -MAXSPEED + pSpeed->LeftValue - pSpeed->RightValue;
					pSpeed->LeftValue = -MAXSPEED;
				}
			} else {
				pSpeed->RightValue = (radiu + r) / radiu * samespeed;
				pSpeed->LeftValue = (radiu - r) / radiu * samespeed;
				if (pSpeed->RightValue > MAXSPEED) {
					pSpeed->LeftValue = MAXSPEED - pSpeed->RightValue + pSpeed->LeftValue;
					pSpeed->RightValue = MAXSPEED;
				}
			}
		}
	}


/************************************************************************/
/*  ĩ������                                                            */
/*	�ɵ�����: maxd, maxe, kk, basespeed, kk								*/
/************************************************************************/
	
	// samespeed
	double basespeed = MAXSPEED;//100+(220-Robot[nRobot].x)/200*20;
	basespeed = Maths::Limit(basespeed, 100, MAXSPEED);
	
	double kk = 0.7;            // ���Ҳ���
	double maxd = 16;        // ��ͳ����뷶Χ
	double maxe = 3.7;        // �켣��Χ
	distE = fabs(dist * sin(angle3));
	//��ͳ�����ܽ������ڳ��Ĺ켣��Χ��,���ŽǶȲ��Ǻܴ�
	if (dist <= maxd
	    && distE <= maxe
	    && IfEndprocess > 1) {
		//	SendMsg(2, " cos");
		
		double anglelimit, anglet1, anglet2, anglet3;
		anglelimit = (150 - Robot[nRobot].x - 5) / (150 - 5) * PI * .2 + PI * .2;
		if (anglelimit > PI * .4)
			anglelimit = PI * .4;
		dbPOINT pt1, pt2;
		pt1.x = pt2.x = PITCH_LENGTH;
		pt1.y = PITCH_WIDTH / 2 + GOAL_WIDTH / 2;
		pt2.y = PITCH_WIDTH / 2 - GOAL_WIDTH / 2;
		
		//��������
		if (angle3 < PI * 0.2 && angle3 >= 0 || angle3 > PI * 1.8 && angle3 < 2 * PI) {
			double anglek;
			anglek = VecPosition::NormalizeAngle2PI(Robot[nRobot].theta - anglerb2target);
			if (anglek > PI)
				anglek -= 2 * PI;
			
			//xue
			anglet1 = VecPosition::NormalizeAngle2PI(Robot[nRobot].theta - Getpt2ptAngle(rbPt, pt1));
			if (anglet1 > PI)
				anglet1 -= 2 * PI;
			anglet2 = VecPosition::NormalizeAngle2PI(Robot[nRobot].theta - Getpt2ptAngle(rbPt, pt2));
			if (anglet2 > PI)
				anglet2 -= 2 * PI;
			if (fabs(anglet1) > fabs(anglet2))
				anglet3 = fabs(anglet2);
			else
				anglet3 = fabs(anglet1);
			
			//
			if (anglet3 < anglelimit || (anglet1 * anglet2 < 0))//xue
			{
				//�����־
				sym[2] = currentOrder[1];
				sym[1] = 1;
				sym[0] = 1;
				
				if (anglek > 0) {
					if (fabs(anglek) < PI / 2)
						pSpeed->LeftValue = (MAXSPEED - basespeed) * (PI / 2 - fabs(anglek)) / PI * 2 + basespeed;
					else
						pSpeed->LeftValue = basespeed;
					
					pSpeed->RightValue = pSpeed->LeftValue * fabs(cos(anglek)) * kk;
				} else {
					
					if (fabs(anglek) < PI / 2)
						pSpeed->RightValue = (MAXSPEED - basespeed) * (PI / 2 - fabs(anglek)) / PI * 2 + basespeed;
					else
						pSpeed->RightValue = basespeed;
					pSpeed->LeftValue = pSpeed->RightValue * fabs(cos(anglek)) * kk;
					
				}
			}
		}
			//��������
		else if (angle3 > PI - PI * 0.2 && angle3 < PI + PI * 0.2) {
			double anglek;
			anglek = VecPosition::NormalizeAngle2PI(Robot[nRobot].theta + PI - anglerb2target);
			if (anglek > PI)
				anglek -= 2 * PI;
			//xue
			anglet1 = VecPosition::NormalizeAngle2PI(Robot[nRobot].theta - Getpt2ptAngle(rbPt, pt1) + PI);
			if (anglet1 > PI)
				anglet1 -= 2 * PI;
			anglet2 = VecPosition::NormalizeAngle2PI(Robot[nRobot].theta - Getpt2ptAngle(rbPt, pt2) + PI);
			if (anglet2 > PI)
				anglet2 -= 2 * PI;
			if (fabs(anglet1) > fabs(anglet2))
				anglet3 = fabs(anglet2);
			else
				anglet3 = fabs(anglet1);
			
			if (anglet3 < anglelimit || (anglet1 * anglet2 < 0)) {
				//�����־
				sym[2] = currentOrder[1];
				sym[1] = 0;
				sym[0] = 1;
				
				if (anglek > 0) {
					if (fabs(anglek) < PI / 2)
						pSpeed->RightValue = (MAXSPEED - basespeed) * (PI / 2 - fabs(anglek)) / PI * 2 + basespeed;
					else
						pSpeed->RightValue = basespeed;
					
					pSpeed->RightValue = -pSpeed->RightValue;
					pSpeed->LeftValue = pSpeed->RightValue * fabs(cos(anglek)) * kk;
				} else {
					if (fabs(anglek) < PI / 2)
						pSpeed->LeftValue = (MAXSPEED - basespeed) * (PI / 2 - fabs(anglek)) / PI * 2 + basespeed;
					else
						pSpeed->LeftValue = basespeed;
					pSpeed->LeftValue = -pSpeed->LeftValue;
					
					pSpeed->RightValue = pSpeed->LeftValue * fabs(cos(anglek)) * kk;
					
				}
				
			}
		}
	}

/************************************************************************/
/*  ĩ��ֱ��                                                            */
/*	�ɵ�����: maxe, angleLimit1,2										*/
/************************************************************************/
	//�Ƕ��ر�õĴ���
	if (IfEndprocess > 0) {
		maxe = 4;
		double angleLimit1 = 0.2 * PI;
		double angleLimit2 = 0.45 * PI;
		
		//��������
		if ((angle3 < angleLimit1 && angle3 >= 0 || angle3 > 2 * PI - angleLimit1 && angle3 < 2 * PI)
		    && fabs(angle1) < PI * 0.45
		    && distE <= maxe &&
		    (angle2 >= 0 && angle2 <= angleLimit2 || angle2 >= 2 * PI - angleLimit2)) {
			double temp;
			temp = Robot[nRobot].y + (PITCH_LENGTH - Robot[nRobot].x) * tan(angle2);
			if (temp >= PITCH_WIDTH / 2 - GOAL_WIDTH / 2 + 1
			    && temp <= PITCH_WIDTH / 2 + GOAL_WIDTH / 2 - 1) {
//				SendMsg(2, "line");
				
				pSpeed->LeftValue = pSpeed->RightValue = MAXSPEED;
				//�����־
				sym[2] = currentOrder[1];
				sym[1] = 1;
				sym[0] = 1;
			}
		}
			//��������
		else if ((angle3 > PI - angleLimit1 && angle3 < PI + angleLimit1)
		         && fabs(angle1) < PI * 0.45
		         && distE <= maxe
		         && angle2 >= PI - angleLimit2 && angle2 <= 2 * PI - angleLimit2) {
			double temp;
			temp = Robot[nRobot].y + (PITCH_LENGTH - Robot[nRobot].x) * tan(angle2);
			if (temp >= PITCH_WIDTH / 2 - GOAL_WIDTH / 2 + 1
			    && temp <= PITCH_WIDTH / 2 + GOAL_WIDTH / 2 - 1) {
//				SendMsg(2, "line");
				
				
				pSpeed->LeftValue = pSpeed->RightValue = -MAXSPEED;
				//�����־
				sym[2] = currentOrder[1];
				sym[1] = 0;
				sym[0] = 1;
			}
		}
	}
}

////////////////// Basic Actions End /////////////////////////////////////////////////////

////////////////// Complex Actions ///////////////////////////////////////////////////////

/************************************************************************/
/*  ���ź���                                                            */
/************************************************************************/
void CDecisionMakingx::Vect_MidShoot(int nRobot) {
	VecPosition posGoal(PITCH_LENGTH, PITCH_WIDTH / 2);
	VecPosition posBall;
	GetToPositionNewPerformance(11, nRobot, posGoal, posBall);
	ToPositionNew(nRobot, posGoal, posBall, MAXSPEED, 0);
	return;
	if (Robot[nRobot].x > m_posBall.GetX()) {
		VecPosition pos;
		double det = 30;
		pos.SetX(m_posBall.GetX() + 100);
		if (m_posBall.GetY() > Robot[nRobot].y)
			pos.SetY(m_posBall.GetY() + 5);
		else
			pos.SetY(m_posBall.GetY() - 5);
		
		if (m_posBall.GetY() < det)
			pos.SetY(m_posBall.GetY() - 3);
		else if (m_posBall.GetY() > PITCH_WIDTH - det)
			pos.SetY(m_posBall.GetY() + 3);
		ToPositionNew(nRobot, pos, m_posBall, 110, 2);
	}
}

/************************************************************************/
/*  �з�������                                                          */
/************************************************************************/
void CDecisionMakingx::Vect_MidShoot1(int nRobot) {
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	VecPosition posGoal(PITCH_LENGTH, PITCH_WIDTH / 2);
	VecPosition posBall = m_posBall;
	
	Line lineBall2Goal = Line::MakeLineFromTwoPoints(posGoal, posBall);
	double L = 0, R = 30;
	VecPosition posShoot = lineBall2Goal.GetPointInLine(posBall, -L);
	Line linePerp = lineBall2Goal.GetPerpendicularLine(posShoot);
	VecPosition posCenter = linePerp.GetPointInLine(posShoot, -R);
	Circle c(posCenter, R);
	Line lineRobot2Shoot = Line::MakeLineFromTwoPoints(posRobot, posShoot);
	VecPosition posMid = (posRobot + posShoot) / 2;
	Line linePerp2 = lineRobot2Shoot.GetPerpendicularLine(posMid);
	VecPosition posSolu[2], posTarget;
	linePerp2.GetCircleIntersectionPoints(c, &posSolu[0], &posSolu[1]);
	if (posSolu[0].GetDistanceTo(posRobot) > posSolu[1].GetDistanceTo(posRobot))
		posTarget = posSolu[1];
	else
		posTarget = posSolu[0];
	
	ToPositionPD(nRobot, posTarget, 100);
	
	SendMsg(3, "%2.2f, %2.2f", posTarget.GetX(), posTarget.GetY());
}

// Բ������
void CDecisionMakingx::Vect_MidShoot2(int nRobot) {
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	VecPosition posGoal(PITCH_LENGTH, PITCH_WIDTH / 2);
	VecPosition posBall = m_posBall;
	
	double L = 5;
	Line lineBall2Goal = Line::MakeLineFromTwoPoints(posBall, posGoal);
	posBall = lineBall2Goal.GetPointInLine(posBall, L *
	                                                ((posBall.GetX() < posGoal.GetX()) ? -1 : 1));
	Line linePerBall2Goal = lineBall2Goal.GetPerpendicularLine(posBall);
	Line lineRobot2Ball = Line::MakeLineFromTwoPoints(posRobot, posBall);
	Line lineMidPerRobot2Ball = lineRobot2Ball.GetPerpendicularLine((posRobot + posBall) / 2);
	VecPosition posCenter = linePerBall2Goal.GetIntersection(lineMidPerRobot2Ball);
	
	double dRadius = (posCenter - posBall).GetMagnitude();
	Circle circle(posCenter, dRadius);
//	double d = (posCenter - posRobot).GetMagnitude();
	
	rbV[nRobot].LeftValue = (dRadius - 4.0);//-(dRadius + 4.1)/dRadius * 100;
	rbV[nRobot].RightValue = (dRadius + 4.0);//-(dRadius - 4.1)/dRadius * 100;

//	EndProcess(1, nRobot, posGoal, m_posBall);
//	rbV[nRobot].LeftValue = -12;//-(dRadius + 4.1)/dRadius * 100;
//	rbV[nRobot].RightValue = 0;//-(dRadius - 4.1)/dRadius * 100;
}

int CDecisionMakingx::Vect_MidShootli(dbROBOTPOSTURE pRobotInford, BallInformation &ball, dbLRWheelVelocity *pSpeed) {
//	pSpeed->LeftValue = pSpeed->RightValue = 0;
//	return 1;
	
	dbPOINT goal, ballPt;
	double delta, angletemp;
	delta = 10;
	ballPt = ball;
	goal.x = PITCH_LENGTH + 5;
	goal.y = PITCH_WIDTH / 2;
	angletemp = Getpt2ptAngle(ballPt, goal);
	//ballPt.x = ballPt.x - delta*sin(angletemp);
	//ballPt.y = ballPt.y - delta*cos(angletemp);
	double deltax;
	deltax = 10;
	if ((ball.x > PITCH_LENGTH - deltax) && (ball.y > PITCH_LENGTH / 2 - GOAL_AREA_WIDTH / 2 - deltax &&
	                                         ball.y < PITCH_LENGTH / 2 + GOAL_AREA_WIDTH / 2 + deltax)) {
		if (ball.y <= PITCH_LENGTH / 2)
			goal.y = ball.y + deltax;
		else
			goal.y = ball.y - deltax;
		goal.x = PITCH_LENGTH + 8;
		ToPositionNewli(&pRobotInford, ballPt, goal, 120, 2, pSpeed);
		return 1;
	}
	ToPositionNewli(&pRobotInford, ballPt, goal, 120, 2, pSpeed);
	/*if(pRobotInford.x>ball.x)
	Vect_MidShoot2(pRobotInford,ball,goal,0,pSpeed);*/
	return 1;
	
}


/************************************************************************/
/* ��λ����                                                             */
/************************************************************************/
void CDecisionMakingx::Wait(int nRobot, VecPosition posTarget, double angle, double speed) {
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	
	if ((posTarget - posRobot).GetMagnitude() > 1) {
#if 0
																																VecPosition* posObstatcles = new VecPosition[m_nPriorityResult[nRobot]+3];
		posObstatcles[0] = posTarget;	// Ŀ��
		posObstatcles[1] = m_posBall;	// ��
		
		int nOb = 1;

		for (int i=0; i<ROBOTNUMBER; i++)
		{
			if (m_nPriorityResult[i] < m_nPriorityResult[nRobot])
			{
				posObstatcles[nOb+1].SetVecPosition(Robot[i].x, Robot[i].y);
				nOb++;
			}
		}
		if (nOb != m_nPriorityResult[nRobot]+1)
			::MessageBox(NULL, "Obstacle Avoid, Priority", "Error", MB_OK);

		ToPositionAvoidObstacles(&Robot[nRobot], posObstatcles, nOb, &rbV[nRobot]);
		
		delete[] posObstatcles;
#else
		ToPositionPD(nRobot, posTarget, speed);
#endif
	} else {
		TurnToAnglePD(nRobot, angle, NOCLOCK);
	}
}

void CDecisionMakingx::Wait(int nRobot, VecPosition posTarget, VecPosition posToward, double speed) {
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	double angle = (posToward - posRobot).GetDirection();
	
	Wait(nRobot, posTarget, angle, speed);
}

/************************************************************************/
/* ֱ�����ź���                                                         */
/************************************************************************/
void CDecisionMakingx::ShootLine(int nRobot) {
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	VecPosition posBall;
	
	int nCycle = GetKickBallPerformance(21, nRobot, posBall);
	
	Line line = Line::MakeLineFromTwoPoints(posRobot, posBall);
	double y = line.GetYGivenX(PITCH_LENGTH);
	VecPosition posOpponentGoalie(Robot[2 * ROBOTNUMBER - 1].x, Robot[2 * ROBOTNUMBER - 1].y);
	
	SendMsg(1, "%d %2.2f", nCycle, y);
	
	if (m_bShootLine[nRobot]
	    || (y > PITCH_WIDTH / 2 - GOAL_WIDTH / 2 - 5
	        && y < PITCH_WIDTH / 2 + GOAL_WIDTH / 2 + 5
	        && posBall.GetX() < PITCH_LENGTH + 10
	        && posBall.GetX() > PITCH_LENGTH / 2
	        && posRobot.GetX() < posBall.GetX()
	        && posRobot.GetX() > PITCH_LENGTH / 3)
			) {
		ToPositionN(nRobot, posBall, MAXSPEED);
		m_bShootLine[nRobot] = TRUE;
		
		m_timerShootLine.Start(10);
	}
	
	
	if ((posRobot - m_posBall).GetMagnitude() < 10
	    || m_posBall.GetX() > PITCH_LENGTH) {
		m_bShootLine[nRobot] = FALSE;
		Turn(nRobot, 90, ANTICLOCK);
	}
}

int CDecisionMakingx::GetShootLinePerformance(int nRobot) {
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	VecPosition posBall;
	
	int nCycle = GetKickBallPerformance(21, nRobot, posBall);
	
	Line line = Line::MakeLineFromTwoPoints(posRobot, posBall);
	double y = line.GetYGivenX(PITCH_LENGTH);
	if (m_bShootLine[nRobot]
	    || (y > PITCH_WIDTH / 2 - GOAL_WIDTH / 2 - 5
	        && y < PITCH_WIDTH / 2 + GOAL_WIDTH / 2 + 5
	        && posBall.GetX() < PITCH_LENGTH + 10
	        && posBall.GetX() > PITCH_LENGTH / 2
	        && posRobot.GetX() < posBall.GetX()
	        && posRobot.GetX() > PITCH_LENGTH / 2)
			) {
		if (!m_bShootLine[nRobot])
			return nCycle;
		else
			return static_cast<int>(nCycle * 0.6);
	} else {
		return -1;
	}
}
/************************************************************************/
/* Ԥ��������                                                         */
/************************************************************************/
void CDecisionMakingx::KickBall(int nRobot) {
	VecPosition posBall;
	VecPosition posTarget;
	if (posBall.GetY() > PITCH_WIDTH / 2)
		posTarget = (30, 54);
	else
		posTarget = (30, 126);
	
	if (m_posBall.GetX() > Robot[nRobot].x) {
		GetKickBallPerformance(21, nRobot, posBall);
		ToPositionN(nRobot, posBall, MAXSPEED);
	} else {
		GetToPositionNewPerformance(11, nRobot, posTarget, posBall);
		ToPositionNew(nRobot, posTarget, posBall, MAXSPEED, 3);
	}
}

/************************************************************************/
/* ��λ����                                                             */
/************************************************************************/
void CDecisionMakingx::RobotReturn2Pt() {
	if (together) {
		RobotReturn2bound();
		return;
	}
	if (round) {
		Round(m_nTestRobot - 1);
		return;
	}
	
	dbPOINT pt[12];
	switch (dmDEG.DEStartMode) {
		case NormalStart: {
			if (dmDEG.DEStartState == Attack) {
				pt[1].x = PITCH_LENGTH / 2 + 8;
				pt[1].y = PITCH_WIDTH / 2;
				pt[2].x = PITCH_LENGTH / 2 - 15;
				pt[2].y = PITCH_WIDTH / 2;
				pt[3].x = PITCH_LENGTH * .25;
				pt[3].y = PITCH_WIDTH * .75;
				pt[4].x = PITCH_LENGTH * .25;
				pt[4].y = PITCH_WIDTH * .25;
				pt[5].x = 5;
				pt[5].y = PITCH_WIDTH / 2;
			} else {
				pt[1].x = PITCH_LENGTH / 2 - 20;
				pt[1].y = PITCH_WIDTH / 2 + 20;
				pt[2].x = PITCH_LENGTH / 2 - 15;
				pt[2].y = PITCH_WIDTH / 2;
				pt[3].x = PITCH_LENGTH * .25;
				pt[3].y = PITCH_WIDTH * .75;
				pt[4].x = PITCH_LENGTH * .25;
				pt[4].y = PITCH_WIDTH * .25;
				pt[5].x = 5;
				pt[5].y = PITCH_WIDTH / 2;
			}
		}
			break;
		case PenaltyKick: {
			if (dmDEG.DEStartState == Defense) {
				pt[1].x = PITCH_LENGTH / 2 + 8;
				pt[1].y = PITCH_WIDTH / 2;
				pt[2].x = PITCH_LENGTH / 2 + 8;
				pt[2].y = PITCH_WIDTH / 2 + 70;
				pt[3].x = PITCH_LENGTH / 2 + 8;
				pt[3].y = PITCH_WIDTH * .25;
				pt[4].x = PITCH_LENGTH / 2 + 8;
				pt[4].y = PITCH_WIDTH * .75;
				pt[5].x = 8;
				pt[5].y = PITCH_WIDTH / 2;
			} else {
				pt[1].x = PITCH_LENGTH - 45;
				pt[1].y = PITCH_WIDTH / 2;
				pt[2].x = PITCH_LENGTH / 2 - 10;
				pt[2].y = PITCH_WIDTH / 2;
				pt[3].x = PITCH_LENGTH / 2 - 10;
				pt[3].y = PITCH_WIDTH * .25;
				pt[4].x = PITCH_LENGTH / 2 - 10;
				pt[4].y = PITCH_WIDTH * .75;
				pt[5].x = 8;
				pt[5].y = PITCH_WIDTH / 2;
			}
		}
			break;
		case GoalKick: {
			if (dmDEG.DEStartState == Defense) {
				pt[1].x = PITCH_LENGTH / 2 - 15;
				pt[1].y = m_posBall.GetY();
				pt[2].x = PITCH_LENGTH / 2 - 30;
				pt[2].y = PITCH_WIDTH / 2;
				pt[3].x = PITCH_LENGTH / 2 - 15;
				pt[3].y = PITCH_WIDTH * .75;
				pt[4].x = PITCH_LENGTH / 2 - 15;
				pt[4].y = PITCH_WIDTH * .25;
				pt[5].x = 5;
				pt[5].y = PITCH_WIDTH / 2;
			} else {
				pt[1].x = 5;
				pt[1].y = PITCH_WIDTH / 2;
				pt[2].x = PITCH_LENGTH / 2 - 15;
				pt[2].y = PITCH_WIDTH / 2 - 40;
				pt[3].x = PITCH_LENGTH * .25;
				pt[3].y = PITCH_WIDTH * .75;
				pt[4].x = PITCH_LENGTH * .25;
				pt[4].y = PITCH_WIDTH * .25;
				pt[5].x = 15;
				pt[5].y = PITCH_WIDTH / 2 + 50;
			}
		}
			break;
		case FreeKick: {
			if (dmDEG.DEStartState == Attack) {
				pt[1].x = PITCH_LENGTH / 2 + 63;
				pt[1].y = PITCH_WIDTH / 2;
				pt[2].x = PITCH_LENGTH / 2 - 15;
				pt[2].y = PITCH_WIDTH / 2;
				pt[3].x = PITCH_LENGTH * .75;
				pt[3].y = PITCH_WIDTH * .75;
				pt[4].x = PITCH_LENGTH * .75;
				pt[4].y = PITCH_WIDTH * .25;
				pt[5].x = 5;
				pt[5].y = PITCH_WIDTH / 2;
			} else {
				pt[1].x = 5;
				pt[1].y = PITCH_WIDTH / 2;
				pt[2].x = 25;
				pt[2].y = PITCH_WIDTH / 2 - 20;
				pt[3].x = 25;
				pt[3].y = PITCH_WIDTH / 2 + 20;
				pt[4].x = PITCH_LENGTH * .15;
				pt[4].y = PITCH_WIDTH / 2 - 45;
				pt[5].x = 15;
				pt[5].y = PITCH_WIDTH / 2 + 45;
			}
		}
			break;
		case FreeBall: {
			if (m_posBall.GetX() < PITCH_LENGTH / 2 && m_posBall.GetY() < PITCH_WIDTH / 2) {
				pt[1].x = 30;
				pt[1].y = 30;
				pt[2].x = PITCH_LENGTH / 2 + 15;
				pt[2].y = PITCH_WIDTH / 2 + 40;
				pt[3].x = 20;
				pt[3].y = PITCH_WIDTH / 2 + 10;
				pt[4].x = 40;
				pt[4].y = PITCH_WIDTH / 2 + 20;
				pt[5].x = 5;
				pt[5].y = PITCH_WIDTH / 2;
			} else if (m_posBall.GetX() < PITCH_LENGTH / 2 && m_posBall.GetY() > PITCH_WIDTH / 2) {
				pt[1].x = 30;
				pt[1].y = PITCH_WIDTH - 30;
				pt[2].x = PITCH_LENGTH / 2 + 15;
				pt[2].y = PITCH_WIDTH - (PITCH_WIDTH / 2 + 15);
				pt[3].x = 20;
				pt[3].y = PITCH_WIDTH - (PITCH_WIDTH / 2 + 10);
				pt[4].x = 40;
				pt[4].y = PITCH_WIDTH - (PITCH_WIDTH / 2 + 20);
				pt[5].x = 5;
				pt[5].y = PITCH_WIDTH / 2;
			} else if (m_posBall.GetX() > PITCH_LENGTH / 2 && m_posBall.GetY() < PITCH_WIDTH / 2) {
				pt[1].x = (PITCH_LENGTH - 80);
				pt[1].y = 30;
				pt[2].x = PITCH_LENGTH - 80;
				pt[2].y = PITCH_WIDTH / 2 + 15;
				pt[3].x = 2 * 156 - (PITCH_LENGTH - 10);
				pt[3].y = 30;
				pt[4].x = 2 * 156 - (PITCH_LENGTH - 20);
				pt[4].y = 60;
				pt[5].x = 5;
				pt[5].y = PITCH_WIDTH / 2;
			} else {
				pt[1].x = (PITCH_LENGTH - 80);//2*156-
				pt[1].y = PITCH_WIDTH - 30;
				pt[2].x = PITCH_LENGTH - 80;
				pt[2].y = PITCH_WIDTH - (PITCH_WIDTH / 2 + 15);
				pt[3].x = 2 * 156 - (PITCH_LENGTH - 10);
				pt[3].y = PITCH_WIDTH - 30;
				pt[4].x = 2 * 156 - (PITCH_LENGTH - 20);
				pt[4].y = PITCH_WIDTH - 60;
				pt[5].x = 5;
				pt[5].y = PITCH_WIDTH / 2;
			}
		}
			break;
		default:
			break;
	}
	
	for (int i = 0; i < ROBOTNUMBER - 1; i++) {
		Wait(i, VecPosition(pt[i + 1].x, pt[i + 1].y), m_posBall, 40);
	}
	Wait(ROBOTNUMBER, VecPosition(pt[ROBOTNUMBER].x, pt[ROBOTNUMBER].y),
	     PI / 2, 40);
	
}

void CDecisionMakingx::RobotReturn2bound() {
	PreProcess();//��ϢԤ����
	VecPosition pos[ROBOTNUMBER];
	
	pos[0].SetVecPosition(PITCH_LENGTH / 2 - 30, 15);
	pos[1].SetVecPosition(PITCH_LENGTH / 2 - 15, 15);
	pos[2].SetVecPosition(PITCH_LENGTH / 2, 15);
	pos[3].SetVecPosition(PITCH_LENGTH / 2 + 15, 15);
	pos[4].SetVecPosition(PITCH_LENGTH / 2 + 30, 15);
	
	for (int i = 0; i < ROBOTNUMBER; i++)
		ToPositionPD(i, pos[i], 40);
}

void CDecisionMakingx::Round(int nRoundRobot) {
	VecPosition pos[12];
	pos[0].SetVecPosition(25, 25);
	pos[1].SetVecPosition(25, PITCH_WIDTH - 25);
	pos[2].SetVecPosition(PITCH_LENGTH - 25, PITCH_WIDTH - 25);
	pos[3].SetVecPosition(PITCH_LENGTH - 25, 25);
	pos[4] = pos[0];
	pos[5] = pos[2];
	pos[6] = pos[3];
	pos[7] = pos[1];
	pos[8].SetVecPosition(25, PITCH_WIDTH / 2);
	pos[9].SetVecPosition(PITCH_LENGTH - 25, PITCH_WIDTH / 2);
	pos[10] = pos[3];
	pos[11].SetVecPosition(PITCH_LENGTH / 2, 25);
	
	VecPosition pRobot[5];
	
	static int k[5] = {0, 0, 0, 0, 0};
	
	{
		pRobot[nRoundRobot].SetVecPosition(Robot[nRoundRobot + 1].x, Robot[nRoundRobot + 1].y);
		
		ToPositionPD(nRoundRobot + 1, pos[k[nRoundRobot]], 30);
		
		if (k[nRoundRobot] > 11) {
			rbV[nRoundRobot + 1].LeftValue = 40;
			rbV[nRoundRobot + 1].RightValue = 36;
		}
		
		if (pRobot[nRoundRobot].GetDistanceTo(pos[k[nRoundRobot]]) < 10) {
			k[nRoundRobot]++;
		}
		
		
		if (k[nRoundRobot] > 12) k[nRoundRobot] = 0;
	}
}
/////////////////// Complex Actions End ////////////////////////////////////////////////////


/////////////////// Goalie ////////////////////////////////////////////////////////////////
void CDecisionMakingx::GoalieAction(int nRobot) {
	int gtError = 2, glError = 2;            // gate corner and goal corner +-
	
	int goToken = 0;    // �������� ��С���ٶ�
	
	BOOL bIsGate = FALSE;    // �Ƿ���켣������
	double speed = gVMAX;
	
	VecPosition posTarget(G_OFFSET + ROBOT_LENGTH, PITCH_WIDTH / 2);

//	double yBall = GetCrossPtWithGLLine();
	
	Line lineBall(ballCharacter.Formu.b, ballCharacter.Formu.a, ballCharacter.Formu.c);
	double yGate = lineBall.GetYGivenX(0);        //��켣�����Ž���
	
	double yBall = lineBall.GetYGivenX(2 * ROBOT_LENGTH + G_OFFSET);
	if (m_posBall.GetX() < 9.5) {
		int y = 1;
	}
	VecPosition posBall;
	posBall = PredictBall(2);//���Ǽ򵥵�ֱ��Ԥ�⡣û����Ԥ������ã����Ǵ���һ��X
	// ������������
	if ((oldBallPt[4].x > oldBallPt[6].x || oldBallPt[5].x > oldBallPt[6].x))                 // GG new add
	{
		// ����������
		if (yGate < PITCH_WIDTH / 2 + GOAL_WIDTH / 2 + 2
		    && yGate > PITCH_WIDTH / 2 - GOAL_WIDTH / 2 - 2) {
			bIsGate = TRUE;
			SendMsg(1, "bIsGate");
		}
	}
	if (m_posBall.GetX() < 19.5) {
		posTarget.SetY(AdjustTarget());//�˴��ǹؼ�
	}
	
	// x�������źܽ���penalty
	if (m_posBall.GetX() < GOAL_AREA_LENGTH + 5) {
		SendMsg(1, "X<15");
		int key;
		key = dmDEG.DEPenaltyDirection;
		if (key == 1)
			SendMsg(1, "X<15");
		//  ����������
		if (bIsGate) {
#ifdef SimuroSot5
			posTarget.SetY(yBall);
#else
			posTarget.SetY(AdjustTarget());//�˴��ǹؼ�
#endif
			
			goToken = 0;//����ֹͣ
			
			SendMsg(1, " 0 posTarget %2.2f", posTarget.GetY());
		} else {
			// �������,վ��С��������
			if (m_posBall.GetY() < PITCH_WIDTH / 2 - PENALTY_AREA_WIDTH / 2) {
				posTarget.SetY(PITCH_WIDTH / 2 - GOAL_AREA_WIDTH / 2 + glError);
				goToken = 1;
				
				SendMsg(1, "1 posTarget %2.2f", posTarget.GetY());
			} else if (m_posBall.GetY() > PITCH_WIDTH / 2 + PENALTY_AREA_WIDTH / 2) {
				posTarget.SetY(PITCH_WIDTH / 2 + GOAL_AREA_WIDTH / 2 - glError);
				goToken = 1;
				
				SendMsg(1, "1 posTarget %2.2f", posTarget.GetY());
			}
				// С�����������
			else if (m_posBall.GetY() < PITCH_WIDTH / 2 - GOAL_WIDTH / 2
			         && m_posBall.GetY() > PITCH_WIDTH / 2 + GOAL_WIDTH / 2) {
				posTarget.SetY(posBall.GetY());
				goToken = 1;
				
				SendMsg(1, "1 posTarget %2.2f", posTarget.GetY());
			}
				// �������С����֮��
			else {
				posTarget.SetY(posBall.GetY());
				
				goToken = 0;
				
				SendMsg(1, "0 posTarget %2.2f", posTarget.GetY());
			}
		}
	}
		// ����<30����
	else if (m_posBall.GetX() < GOAL_AREA_LENGTH + 10) {
		SendMsg(1, "X < 30");
		
		if (bIsGate) {
#ifdef SimuroSot5
			posTarget.SetY(yBall);
#else
			posTarget.SetY(AdjustTarget());
#endif
			
			goToken = 0;
			
			SendMsg(1, "0 posTarget %2.2f", posTarget.GetY());
		} else {
			// С������
			if (m_posBall.GetY() < PITCH_WIDTH / 2 - GOAL_AREA_WIDTH / 2) {
				posTarget.SetY(PITCH_WIDTH / 2 - GOAL_WIDTH / 2 + gtError);
				goToken = 1;
				
				SendMsg(1, "1 posTarget %2.2f", posTarget.GetY());
			} else if (m_posBall.GetY() > PITCH_WIDTH / 2 + GOAL_AREA_WIDTH / 2) {
				posTarget.SetY(PITCH_WIDTH / 2 + GOAL_WIDTH / 2 - gtError);
				goToken = 1;
				
				SendMsg(1, "1 posTarget %2.2f", posTarget.GetY());
			} else {
				posTarget.SetY(posBall.GetY());
				goToken = 0;
				
				SendMsg(1, "0 posTarget %2.2f", posTarget.GetY());
			}
		}
	}
		// ����<45����
	else if (m_posBall.GetX() < PENALTY_AREA_LENGTH + 10) {
		SendMsg(1, "X < 45");
		
		if (bIsGate) {
#ifdef SimuroSot5
			posTarget.SetY(yBall);
#else
			posTarget.SetY(AdjustTarget());
#endif
			goToken = 0;
			
			SendMsg(1, "0 posTarget %2.2f", posTarget.GetY());
		} else {
			// ��������
			if (ball.y < PITCH_WIDTH / 2 - GOAL_WIDTH / 2) {
				posTarget.SetY(PITCH_WIDTH / 2 - GOAL_WIDTH / 2 + 2 * gtError);
				goToken = 2;
				
				SendMsg(1, "2 posTarget %2.2f", posTarget.GetY());
			} else if (ball.y > PITCH_WIDTH / 2 + GOAL_WIDTH / 2) {
				posTarget.SetY(PITCH_WIDTH / 2 + GOAL_WIDTH / 2 - 2 * gtError);
				goToken = 2;
				
				SendMsg(1, "2 posTarget %2.2f", posTarget.GetY());
			} else {
				posTarget.SetY(posBall.GetY());
				goToken = 0;
				
				SendMsg(1, "0 posTarget %2.2f", posTarget.GetY());
			}
		}
	}
		// ����<65����
	else if (m_posBall.GetX() < 65) {
		SendMsg(1, "X < 65");
		if (bIsGate) {
#ifdef SimuroSot5
			posTarget.SetY(yBall);
#else
			posTarget.SetY(AdjustTarget());
#endif
			
			goToken = 0;
			
			SendMsg(1, "0 posTarget %2.2f", posTarget.GetY());
		} else {
			if (m_posBall.GetY() < PITCH_WIDTH / 2 - GOAL_WIDTH / 2) {
				posTarget.SetY(PITCH_WIDTH / 2 - GOAL_WIDTH / 2 + 3 * gtError);
				goToken = 2;
				
				SendMsg(1, "2 posTarget %2.2f", posTarget.GetY());
			} else if (m_posBall.GetY() > PITCH_WIDTH / 2 + GOAL_WIDTH / 2) {
				posTarget.SetY(PITCH_WIDTH / 2 + GOAL_WIDTH / 2 - 3 * gtError);
				goToken = 2;
				
				SendMsg(1, "2 posTarget %2.2f", posTarget.GetY());
			} else {
				posTarget.SetY(posBall.GetY());
				goToken = 2;
				
				SendMsg(1, "2 posTarget %2.2f", posTarget.GetY());
			}
		}
	}
		//  ����>65����
	else {
		SendMsg(1, "X > 65");
		
		if (bIsGate) {
#ifdef SimuroSot5
			posTarget.SetY(yBall);
#else
			posTarget.SetY(AdjustTarget());
#endif
			
			goToken = 2;
			
			SendMsg(1, "2 posTarget %2.2f", posTarget.GetY());
		} else {
			if (ball.y < PITCH_WIDTH / 2 - GOAL_WIDTH / 2) {
				posTarget.SetY(PITCH_WIDTH / 2 - GOAL_WIDTH / 2 + 4 * gtError);
				goToken = 2;
				
				SendMsg(1, "2 posTarget %2.2f", posTarget.GetY());
			} else if (ball.y > PITCH_WIDTH / 2 + GOAL_WIDTH / 2) {
				posTarget.SetY(PITCH_WIDTH / 2 + GOAL_WIDTH / 2 - 4 * gtError);
				goToken = 2;
				
				SendMsg(1, "2 posTarget %2.2f", posTarget.GetY());
			} else {
				posTarget.SetY(PITCH_WIDTH / 2);
				goToken = 2;
				
				SendMsg(1, "2 posTarget %2.2f", posTarget.GetY());
			}
		}
	}
/*	// ����������ڣ�����λ�ã���Ƕ�
	if (bIsGate)
	{
		if (ballCharacter.angle > PI/2)
		{
			posTarget.SetY(posTarget.GetY() - ROBOT_LENGTH);
		}
		else
		{
			posTarget.SetY(posTarget.GetY() + ROBOT_LENGTH);
		}
	}
*/
	if (m_posBall.GetX() <= 15) {
		posTarget.SetY(m_posBall.GetY());
	}
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	double dist = (posRobot - posTarget).GetMagnitude();
	double distBall = (posRobot - m_posBall).GetMagnitude();
	
	//speed = ballCharacter.velocity * 60;
	//speed = ballCharacter.velocity * 60;
	speed = ballCharacter.velocity * 70;
	switch (goToken) {
		case 0:
			if (dist < 2)
				ToPositionPDGoal(nRobot, posTarget, speed, 0);
			else
				ToPositionPDGoal(nRobot, posTarget, speed, 50);
			break;
		case 1:
			if (dist < 2)
				ToPositionPDGoal(nRobot, posTarget, speed, 0);
			else
				ToPositionPDGoal(nRobot, posTarget, speed, 30);
			break;
		case 2:
			ToPositionPDGoal(nRobot, posTarget, speed, 30);
			break;
/*	case	3:
		if(dist < 2)
			ToPositionPDGoal(pRobot, posTarget,speed,0.0,pSpeed);
		else
		{
			if(distBall > 60)
			{
				if(ballCharacter.velocity < 8)
					speed = gVMAX - 20.0;
				else
					speed = gVMAX - 15.0;

				if(dist > 20)
					speed = gVMAX - 10.0;
			}
			else
			{
				if(ballCharacter.velocity > 12.5
					|| dist > 13)
					speed = gVMAX;
			}

			if(ballCharacter.velocity > 15
				&& bIsGate)
				speed = gVMAX ;
			if(m_posBall.GetX() < 10
				&& bIsGate)
				speed = gVMAX ;

			ToPositionPDGoal(pRobot, posTarget,speed,40,pSpeed);
		}
		break;
	case	4:
		MoveToPt(pRobot,  m_posBall, speed, pSpeed);
		break;
*/    default:
			::MessageBox(NULL, "GoalAction No goToken", "Error", MB_OK);
			break;
	}
	
	dbLRWheelVelocity *pSpeed = &rbV[nRobot];

/*	if( dist < 5)
	{
		if (fabs(fabs(Robot[nRobot].theta) - PI/2) > PI/180.*1.)
		{
			if(Robot[nRobot].theta < 0)
				TurnToAnglePD(&Robot[nRobot],-PI/2,NOCLOCK,pSpeed);
			else
				TurnToAnglePD(&Robot[nRobot],PI/2,NOCLOCK,pSpeed);
		}
	}
*/
	
	VecPosition posBallTarget;
	if (m_posBall.GetY() > PITCH_WIDTH / 2)
		posBallTarget.SetVecPosition(2 * ROBOT_LENGTH + G_OFFSET, PITCH_WIDTH - 30);
	else
		posBallTarget.SetVecPosition(2 * ROBOT_LENGTH + G_OFFSET, 30);
	
	if (m_posBall.GetX() < 2 * ROBOT_LENGTH + G_OFFSET) {
		// �������
		if (m_posBall.GetY() > PITCH_WIDTH / 2 + PENALTY_AREA_WIDTH / 2) {
			pSpeed->LeftValue = pSpeed->RightValue = 0;
			SendMsg(1, "Turn0");
			
			posBallTarget.SetVecPosition(ROBOT_LENGTH + G_OFFSET, PITCH_WIDTH / 2 + PENALTY_AREA_WIDTH / 2 - 5);
			//	ToPositionPD(pRobot, posBallTarget, MAXSPEED, pSpeed);
		} else if (m_posBall.GetY() < PITCH_WIDTH / 2 - PENALTY_AREA_WIDTH / 2) {
			pSpeed->LeftValue = pSpeed->RightValue = 0;
			SendMsg(1, "Turn0");
			
			posBallTarget.SetVecPosition(ROBOT_LENGTH + G_OFFSET, PITCH_WIDTH / 2 - PENALTY_AREA_WIDTH / 2 + 5);
			//	ToPositionPD(pRobot, posBallTarget, MAXSPEED, pSpeed);
		}
			
			// ����С��������
		else if (m_posBall.GetY() > PITCH_WIDTH / 2 + GOAL_WIDTH / 2) {
			SendMsg(1, "Turn1");
			
			posBallTarget.SetVecPosition((posRobot.GetX() + m_posBall.GetX()) / 2, m_posBall.GetY());
			ToPositionPD(nRobot, posBallTarget, MAXSPEED);
		} else if (m_posBall.GetY() < PITCH_WIDTH / 2 - GOAL_WIDTH / 2) {
			SendMsg(1, "Turn2");
			
			posBallTarget.SetVecPosition((posRobot.GetX() + m_posBall.GetX()) / 2, m_posBall.GetY());
			ToPositionPD(nRobot, posBallTarget, MAXSPEED);
		}
			// ������
		else {
			// �򼸺���������
			if (m_posBall.GetX() < BALL_SIZE) {
				
				//key = dmDEG.DEPenaltyDirection;
				//ToPositionNew(nRobot, posBallTarget, m_posBall, 50, 2);
				ToPositionNew(nRobot, posBallTarget, m_posBall, 70, 2);
				SendMsg(1, "ToPositionNew1");
			}
			
			// ����Ա����ǰ��, ����
			if (m_posBall.GetX() < BALL_SIZE + 1
			    && m_posBall.GetX() + BALL_SIZE < posRobot.GetX() - ROBOT_LENGTH + 2) {
				if (m_posBall.GetDistanceTo(posRobot) < ROBOT_LENGTH + BALL_SIZE) {
					VecPosition posBallTarget(posRobot.GetX(), m_posBall.GetY());
					ToPositionPDGoal(nRobot, posBallTarget, 80, 20);
					SendMsg(1, "Huqiu");
				} else {
					//ToPositionNew(nRobot, posBallTarget, m_posBall, 50, 2);
					ToPositionNew(nRobot, posBallTarget, m_posBall, 70, 2);
					SendMsg(1, "ToPositionNew2");
				}
				
			}
			
			// ��������Աǰ�棬��ת�ƻ���
			if (m_posBall.GetX() - BALL_SIZE > posRobot.GetX() + ROBOT_LENGTH - 2
			    && m_posBall.GetX() - BALL_SIZE < posRobot.GetX() + ROBOT_LENGTH
			    && m_posBall.GetY() < posRobot.GetY() + ROBOT_LENGTH + 1
			    && m_posBall.GetY() > posRobot.GetY() - ROBOT_LENGTH - 1) {
				if (m_posBall.GetY() > PITCH_WIDTH / 2)
					Turn(nRobot, MAXSPEED, ANTICLOCK);
				else
					Turn(nRobot, MAXSPEED, CLOCKWISE);
				
				SendMsg(1, "Turn");
			}
		}
	}
	
	SendMsg(1, "Final posTarget %2.2f %2.2f %2.2f %2.2f %2.2f",
	        posTarget.GetX(), posTarget.GetY(), speed, pSpeed->LeftValue, pSpeed->RightValue);
/*	////////////////     ײǽ����     /////////////////////////////
	// ������
	if(Robot[nRobot].y > PITCH_WIDTH/2 + GOAL_WIDTH/2
		|| Robot[nRobot].y < PITCH_WIDTH/2 - GOAL_WIDTH/2 )
	{
		double theta = VecPosition::NormalizeAngle2PI(Robot[nRobot].theta);
		if(theta < PI/6 || (theta > PI && theta - PI < PI/6))
			TurnToAnglePD(pRobot,PI/2,ANTICLOCK,pSpeed);
		else if((2*PI - theta) < PI/6 || (theta < PI && PI - theta < PI/6))
			TurnToAnglePD(pRobot,PI/2,CLOCKWISE,pSpeed);
	}

	////////////////////  ��������   ////////////////////////////////
	if(distBall < 8)
	{
		if(m_posBall.GetX() > Robot[nRobot].x
			&& fabs(ball.y - Robot[nRobot].y) < 8 )
		{
			if(Robot[nRobot].theta > PI)
			{
				if(m_posBall.GetY() > Robot[nRobot].y)
					TurnToAnglePD(pRobot,PI/2,CLOCKWISE,pSpeed);
				else
					TurnToAnglePD(pRobot,PI/2,ANTICLOCK,pSpeed);
			}
			else
			{
				if(m_posBall.GetY() > Robot[nRobot].y)
					TurnToAnglePD(pRobot,3*PI/2,CLOCKWISE,pSpeed);
				else
					TurnToAnglePD(pRobot,3*PI/2,ANTICLOCK,pSpeed);
			}
		}
	}
*/
}

int CDecisionMakingx::TurnToAnglePDli(dbROBOTPOSTURE *pRobot, double dbAngle, int clock, dbLRWheelVelocity *pSpeed)
//����������ʹС���ķ����Ը�����ʱ�ӷ������ת����Ҫ��ĽǶȷ���
//RobotΪС����λ����Ϣ��AngleΪ��Ҫת��ĽǶȣ�SpeedΪ���ص���������
//pPDΪ������΢�ֵ��ڲ����ṹ
{
	double Difference, SameSpeed;
	int Quadrant;
	Difference = pRobot->theta - dbAngle;
	Difference = cn_AngleTrim2PI(Difference);
	if (Difference < m_AngleParameter.AngleError)//�ж��Ƿ��ڽǶ������֮��
	{
		pSpeed->LeftValue = 0.;
		pSpeed->RightValue = 0.;
		return 1;
	}
	if (clock == ANTICLOCK)
		Difference = 2 * PI - Difference;
	else if (clock == NOCLOCK) {
		if (Difference >= 0 && Difference < PI / 2)//�жϽǶȲ���������
			Quadrant = 1;
		else if (Difference >= PI / 2 && Difference < PI) {
			Quadrant = 2;
			Difference = PI - Difference;
		} else if (Difference >= PI && Difference < 3 * PI / 2) {
			Quadrant = 3;
			Difference = Difference - PI;
		} else {
			Quadrant = 4;
			Difference = 2 * PI - Difference;
		}
	}
	//�˴�����PD����
	SameSpeed = m_AngleParameter.Kp * Difference + m_AngleParameter.Kd * (Difference - m_Front);
	if (SameSpeed > m_AngleParameter.MaxAngleSpeed)
		SameSpeed = m_AngleParameter.MaxAngleSpeed;
	m_Front = Difference;
	if (clock == CLOCKWISE) {
		pSpeed->LeftValue = SameSpeed;
		pSpeed->RightValue = -SameSpeed;
	} else if (clock == ANTICLOCK) {
		pSpeed->LeftValue = -SameSpeed;
		pSpeed->RightValue = SameSpeed;
	} else {
		switch (Quadrant) {
			case 1://˳ʱ����ת
			case 3: {
				pSpeed->LeftValue = SameSpeed;
				pSpeed->RightValue = -SameSpeed;
				break;
			}
			case 2://��ʱ����ת
			case 4: {
				pSpeed->LeftValue = -SameSpeed;
				pSpeed->RightValue = SameSpeed;
				break;
			}
		}
	}
	return 1;
}

int CDecisionMakingx::ToPositionPDli(dbROBOTPOSTURE *pROBOTPOSTURE, dbPOINT Target, double same_speed, double end_speed,
                                     dbLRWheelVelocity *pLRWheelVelocity) {                                                                             //vBase��ʹС�����ﶨ��ʱ����һ�����ٶ�
	int clock_sign, move_sign;
	double theta, theta_e1;//e1Ϊ��ǰ�Ƕ����
	static double theta_e2 = 0;//e2Ϊ��һ���ڽǶ����
	double dx, dy, dx1, dy1, distance;
	double speed;
	
	
	//����任����С������Ϊԭ�㣬С������Ϊy��
	dx1 = Target.x - pROBOTPOSTURE->x;
	dy1 = Target.y - pROBOTPOSTURE->y;
	dx = dx1 * cos(pROBOTPOSTURE->theta - PI / 2) + dy1 * sin(pROBOTPOSTURE->theta - PI / 2);
	dy = -dx1 * sin(pROBOTPOSTURE->theta - PI / 2) + dy1 * cos(pROBOTPOSTURE->theta - PI / 2);
	
	distance = sqrt(dx * dx + dy * dy);//Ϊ��ʹ������ʱ����һ�����ٶ�,���������ķ���
	if (distance > m_MoveParameter.max_distance)
		speed = same_speed;
	else
		speed = distance / m_MoveParameter.max_distance * same_speed;
	if (speed < end_speed) speed = end_speed;
	theta = atan2(dy, dx);
	if (fabs(fabs(theta) - PI / 2) > m_AngleParameter.MaxAngle) {
		TurnToPointPDli(pROBOTPOSTURE, Target, NOCLOCK, pLRWheelVelocity);
		return 0;
	}
	
	theta = cn_AngleTrim2PI(theta);
	if (theta <= PI / 2)//��һ����
	{
		move_sign = 1;
		clock_sign = 1;
		theta_e1 = PI / 2 - theta;
	} else if (theta <= PI)//�ڶ�����
	{
		move_sign = 1;
		clock_sign = -1;
		theta_e1 = theta - PI / 2;
	} else if (theta <= 3 * PI / 2)//��������
	{
		move_sign = -1;
		clock_sign = 1;
		theta_e1 = 3 * PI / 2 - theta;
	} else//��������
	{
		move_sign = -1;
		clock_sign = -1;
		theta_e1 = theta - 3 * PI / 2;
	}
	
	pLRWheelVelocity->LeftValue = speed * move_sign + clock_sign * (m_MoveParameter.kp4pospd * theta_e1 +
	                                                                m_MoveParameter.kd4pospd * (theta_e1 - theta_e2));
	pLRWheelVelocity->RightValue = speed * move_sign - clock_sign * (m_MoveParameter.kp4pospd * theta_e1 +
	                                                                 m_MoveParameter.kd4pospd * (theta_e1 - theta_e2));
	
	//���汾���ڽǶ�����һ������΢����
	theta_e2 = theta_e1;
	return 0;
}

double CDecisionMakingx::TNparali(RobotInford myrobot, dbPOINT ball, dbPOINT goal, double dist, double angle) {
	double tempp;
	tempp = 0;
	/*if(dist>200)
	tempp = 0.0;
	else if(dist>100)
	tempp = 0.0;
	else if(dist>70)
	tempp = 0.0;
	else if(dist>50)
	tempp = 0.0;
	else if(dist > 40)
	tempp = 0.0;
	else if(dist >30)
	tempp = 0.0;
	else if(dist >20)
	tempp = 1;
	else if(dist >15)
	tempp = 1;
	else if(dist >10)
	tempp = 1;
	else
	tempp = 1;*/
	/////////////////////
	/*	double maxdist,mindist;
	maxdist = 50;
	mindist = 13;
	if(dist<=maxdist&&dist>=mindist)
	tempp = (maxdist - dist)/(maxdist-mindist);
	else if(dist>maxdist)
	tempp = 0;*/
	double distlimit1, distlimit2, ktheta, thetat2b, thetab2r, deltheta, anglelimit1, anglelimit2;
	dbPOINT rpt;
	rpt.x = myrobot.x;
	rpt.y = myrobot.y;
	anglelimit1 = PI * 0.5;
	anglelimit2 = PI * .12;
	distlimit1 = 200;
	distlimit2 = 10;
	thetat2b = Getpt2ptAngle(goal, ball);
	thetab2r = Getpt2ptAngle(ball, rpt);
	deltheta = cn_AngleTrim2PI(thetat2b - thetab2r);
	if (deltheta > PI)
		deltheta -= 2 * PI;
	ktheta = (distlimit1 - distRobot2Pt(myrobot, ball)) / (distlimit1 - distlimit2);
	if (ktheta > 1)
		ktheta = 1;
	if (ktheta < 0)
		ktheta = 0;
	if (dist < distlimit1)
		tempp = ktheta * (angle - anglelimit2) / (anglelimit1 - anglelimit2) * PI / 2;
	if (tempp > PI / 2)
		tempp = PI / 2;
	if (tempp < 0)
		tempp = 0;
	return tempp;
}


int
CDecisionMakingx::ToPositionNewli(dbROBOTPOSTURE *robot, ballInformation ball, dbPOINT directionpt, double samespeed,
                                  int IfEndprocess, dbLRWheelVelocity *pSpeed) {
	int vemax;
	vemax = MAXSPEED;
	double dist;
	double Dx, Dy;
	double Anglerb2target;
	
	dbPOINT ballPt;
	double delta, angletemp;
	delta = 6.5;
	ballPt = ball;
	angletemp = Getpt2ptAngle(ballPt, directionpt);
	//ball.x = ball.x - delta*sin(angletemp);
	//ball.y = ball.y + delta*cos(angletemp);
	
	robot->y = 180 - robot->y;
	robot->theta = 2 * PI - robot->theta;
	ball.y = 180 - ball.y;
	directionpt.y = 180 - directionpt.y;
	robot->theta = cn_AngleTrim2PI(robot->theta);
	//��������˵�Ŀ���ĽǶ�
	Dx = ball.x - robot->x;
	Dy = ball.y - robot->y;
	dist = sqrt(Dx * Dx + Dy * Dy);
	
	
	double kp4new;
	kp4new = 100.0;
	Anglerb2target = atan2(Dy, Dx);
	Anglerb2target = cn_AngleTrim2PI(Anglerb2target);
	//����targetpt��directionpt�ĽǶ�
	double Angletpt2dpt;
	Dx = directionpt.x - ball.x;
	Dy = directionpt.y - ball.y;
	Angletpt2dpt = atan2(Dy, Dx);
	Angletpt2dpt = cn_AngleTrim2PI(Angletpt2dpt);
	//�������ߵĲ�ֵ
	double angle;
	angle = Angletpt2dpt - Anglerb2target;
	angle = cn_AngleTrim2PI(angle);
	if (angle > PI)
		angle -= 2 * PI;
	double sign;
	if (angle > 0)
		sign = 1;
	else
		sign = -1;
	//�����¸����ڵ�Ŀ��Ƕ�
	double disiredAngle;
	disiredAngle = Anglerb2target - sign * TNparali(*robot, ball, directionpt, distRobot2Pt(*robot, ball), fabs(angle));
	disiredAngle = cn_AngleTrim2PI(disiredAngle);
	//����Ƕ�ƫ��
	double Angle_e;
	Angle_e = disiredAngle - robot->theta;
	Angle_e = cn_AngleTrim2PI(Angle_e);
	//�����������ٶȲ������������ٶ�
	
	double ka;
	ka = samespeed;
	double speed_e;
	if (Angle_e <= PI / 2)//�Ƕ�ƫ���ڵ�һ����
	{
		speed_e = kp4new * Angle_e;
		if (fabs(Angle_e > PI / 2))
			samespeed = 0;
		else
			samespeed = ka * (PI / 2 - fabs(Angle_e)) / PI * 2;
		
		pSpeed->LeftValue = samespeed + speed_e / 2;
		pSpeed->RightValue = samespeed - speed_e / 2;
	} else if (Angle_e <= PI) {
		speed_e = kp4new * (PI - Angle_e);
		if (fabs(PI - Angle_e) > PI / 2)
			samespeed = 0;
		else
			samespeed = ka * (PI / 2 - fabs(PI - Angle_e)) / PI * 2;
		
		pSpeed->LeftValue = samespeed + speed_e / 2;
		pSpeed->RightValue = samespeed - speed_e / 2;
		pSpeed->LeftValue = -pSpeed->LeftValue;
		pSpeed->RightValue = -pSpeed->RightValue;
	} else if (Angle_e < 3 * PI / 2) {
		speed_e = kp4new * (Angle_e - PI);
		if (fabs(Angle_e - PI) > PI / 3)
			samespeed = 0;
		else
			samespeed = ka * (PI / 2 - fabs(Angle_e - PI)) / PI * 2;
		
		pSpeed->LeftValue = samespeed - speed_e / 2;
		pSpeed->RightValue = samespeed + speed_e / 2;
		pSpeed->LeftValue = -pSpeed->LeftValue;
		pSpeed->RightValue = -pSpeed->RightValue;
	} else {
		speed_e = kp4new * (2 * PI - Angle_e);
		if (fabs(2 * PI - Angle_e) > PI / 2)
			samespeed = 0;
		else
			samespeed = ka * (PI / 2 - fabs(2 * PI - Angle_e)) / PI * 2;
		
		pSpeed->LeftValue = samespeed - speed_e / 2;
		pSpeed->RightValue = samespeed + speed_e / 2;
	}
	
	if (pSpeed->LeftValue > vemax) {
		pSpeed->LeftValue = vemax;
		pSpeed->RightValue = pSpeed->LeftValue - fabs(speed_e);
	}
	if (pSpeed->LeftValue < -vemax) {
		pSpeed->LeftValue = -vemax;
		pSpeed->RightValue = pSpeed->LeftValue + fabs(speed_e);
	}
	if (pSpeed->RightValue < -vemax) {
		pSpeed->RightValue = -vemax;
		pSpeed->LeftValue = pSpeed->RightValue + fabs(speed_e);
	}
	if (pSpeed->RightValue > vemax) {
		pSpeed->RightValue = vemax;
		pSpeed->LeftValue = pSpeed->RightValue - fabs(speed_e);
	}
	//�޸ĳ���Ȼ����ϵ
	ball = ballPt;
	robot->y = 180 - robot->y;
	robot->theta = 2 * PI - robot->theta;
	ball.y = 180 - ball.y;
	directionpt.y = 180 - directionpt.y;
	robot->theta = cn_AngleTrim2PI(robot->theta);
	if (IfEndprocess)
		EndProcessli(robot, directionpt, ball, pSpeed);
	return 1;
}

int CDecisionMakingx::EndProcessli(dbROBOTPOSTURE *pRobot, dbPOINT shoot_target, dbPOINT ballPt,
                                   dbLRWheelVelocity *pSpeed) {
	
	double dist, distE, anglerb2ball, anglerb2target, angle1, angle2, angle3, angle4, angleball2target, angle5, maxe, maxd, maxspeed;
	dbPOINT rbPt, EGoal;
	maxspeed = 110;
	EGoal.x = PITCH_LENGTH + 3;
	EGoal.y = PITCH_WIDTH / 2;
	maxd = 3;//10;
	maxe = 0.5;//2.9;
	dist = distRobot2Pt(*pRobot, ballPt);//������С���ľ���
	rbPt.x = pRobot->x;
	rbPt.y = pRobot->y;
	anglerb2ball = Getpt2ptAngle(rbPt, ballPt);//С��ָ����ķ���
	anglerb2target = Getpt2ptAngle(rbPt, EGoal);//С��ָ��Ŀ��ķ���
	angleball2target = Getpt2ptAngle(ballPt, shoot_target);//��Ŀ�귽��
	angle1 = cn_AngleTrim2PI(anglerb2ball - anglerb2target);
	
	if (angle1 > PI && angle1 <= 2 * PI)
		angle1 = angle1 - 2 * PI;
	
	angle2 = cn_AngleTrim2PI(pRobot->theta);
	angle3 = cn_AngleTrim2PI(angle2 - anglerb2ball);
	
	//ĩ��Բ��
	double radiu;
	double samespeed = 40.0;
//		double theta_E,theta1,theta2;
	LINEFORMULATION line1, line2, line3;
	dbPOINT tempPt1, tempPt2, tempPt3;
	tempPt1.x = pRobot->x;
	tempPt1.y = pRobot->y;
	StdLineForm(tempPt1, pRobot->theta, &line1);//��бʽ�ó�����ֱ�߷���L1
	cn_PointPerpendLine(tempPt1, &line1, &line2, &tempPt2);//������L1�Ĵ��߷���L2
	StdLineForm(tempPt1, ballPt, &line1);//�������߷���L3
	tempPt2.x = (pRobot->x + ballPt.x) / 2;
	tempPt2.y = (pRobot->y + ballPt.y) / 2;//�����е�c
	cn_PointPerpendLine(tempPt2, &line1, &line3, &tempPt3);//��c��L3�Ĵ��߷���
	cn_2LinesCrossPoint(&line2, &line3, &tempPt3);//L2��L3�Ľ��㼴Բ��
	//xue
	double angle8;
	dbPOINT roundpt;
	roundpt = tempPt3;
	angle8 = cn_AngleTrim2PI(angle2 - Getpt2ptAngle(rbPt, ballPt));
	//	BOOL sign1;
	
	
	
	radiu = cn_2PointsDist(tempPt3, ballPt);
	StdLineForm(tempPt3, ball, &line1);//Բ�������߷���L4
	cn_PointPerpendLine(ball, &line1, &line2, &tempPt2);//������L4�Ĵ��߷���L5,L5��Ϊ����ʱ������
	
	
	///xue
	angle4 = angle2 - anglerb2ball;
	
	if (rbPt.y < -(line2.a * rbPt.x + line2.c) / line2.b) {
		if (angle8 <= PI) {
			angle5 = cn_AngleTrim2PI(anglerb2ball - angle4);
		} else {
			angle5 = cn_AngleTrim2PI(anglerb2ball + PI - angle4);
		}
	} else {
		if (angle8 <= PI) {
			angle5 = cn_AngleTrim2PI(anglerb2ball + PI - angle4);
		} else {
			angle5 = cn_AngleTrim2PI(anglerb2ball - angle4);
		}
	}
	
	if (angle5 > PI)
		angle5 -= 2 * PI;
	dbPOINT pt1, pt2;
	pt1.x = PITCH_LENGTH;
	pt1.y = PITCH_WIDTH / 2 + 20;
	pt2.x = PITCH_LENGTH;
	pt2.y = PITCH_WIDTH / 2 - 20;
	double angle6, angle7;
	angle6 = Getpt2ptAngle(ballPt, pt1);
	if (angle6 > PI)
		angle6 -= 2 * PI;
	angle7 = Getpt2ptAngle(ballPt, pt2);
	if (angle7 > PI)
		angle7 -= 2 * PI;
	//xue
	
	double y = -(line2.a * 150 + line2.c) / line2.b;
	if (angle5 > angle7 && angle5 < angle6 && angle2 != anglerb2ball && line2.b != 0 && rbPt.x < ballPt.x &&
	    ball.x < EGoal.x)//�ǶȲ���һ����Χʱ��Բ�켣ǰ��
	{
		//xue4.25
		
		samespeed = radiu / 150 * 40 + 50;
		if (samespeed > 110)
			samespeed = 110;
		//xue 4.26
		double angleround;
		angleround = cn_AngleTrim2PI(Getpt2ptAngle(roundpt, rbPt) - Getpt2ptAngle(roundpt, ballPt));
		
		if (rbPt.y < -(line2.a * rbPt.x + line2.c) / line2.b && angleround < PI * 1.2) {
			if (angle8 <= PI) {
				pSpeed->LeftValue = (radiu + 4) / radiu * samespeed;
				pSpeed->RightValue = (radiu - 4) / radiu * samespeed;
			} else {
				pSpeed->RightValue = -(radiu + 4) / radiu * samespeed;
				pSpeed->LeftValue = -(radiu - 4) / radiu * samespeed;
			}
		} else if (rbPt.y >= -(line2.a * rbPt.x + line2.c) / line2.b && (2 * PI - angleround) < PI * 1.2)//xue 4.26
		{
			if (angle8 <= PI) {
				pSpeed->LeftValue = -(radiu + 4) / radiu * samespeed;
				pSpeed->RightValue = -(radiu - 4) / radiu * samespeed;
			} else {
				pSpeed->RightValue = (radiu + 4) / radiu * samespeed;
				pSpeed->LeftValue = (radiu - 4) / radiu * samespeed;
			}
		}
	}
	
	distE = fabs(dist * sin(angle3));
	//��ͳ�����ܽ������ڶԷ�����������������󣬳�������������ұ߽�Ľ����ڶԷ����Ÿ���
	/*	if(dist<=maxd&&pRobot->x>CENTER_X+PITCH_LENGTH/4&&pRobot->x<ballPt.x&&fabs(angle1)<PI*0.45)
			TurnTo(pRobot,ball,EGoal,124,pSpeed);*/
	//��ͳ�����ܽ������ڳ��Ĺ켣��Χ��,���ŽǶȲ��Ǻܴ�
	if (dist <= maxd && distE <= maxe) {
		//xue
		double anglelimit, anglet1, anglet2, anglet3, kk;
		kk = .8;
		anglelimit = (150 - pRobot->x - 5) / (150 - 5) * PI * .2 + PI * .2;
		if (anglelimit > PI * .4)
			anglelimit = PI * .4;
		dbPOINT pt1, pt2;
		pt1.x = pt2.x = PITCH_LENGTH;
		pt1.y = PITCH_WIDTH / 2 + 20;
		pt2.y = PITCH_WIDTH / 2 - 20;
		//
		
		//��������
		if (angle3 < PI * 0.2 && angle3 >= 0 || angle3 > PI * 1.8 && angle3 < 2 * PI) {
			double anglek;
			anglek = cn_AngleTrim2PI(pRobot->theta - anglerb2target);
			if (anglek > PI)
				anglek -= 2 * PI;
			
			//xue
			anglet1 = cn_AngleTrim2PI(pRobot->theta - Getpt2ptAngle(rbPt, pt1));
			if (anglet1 > PI)
				anglet1 -= 2 * PI;
			anglet2 = cn_AngleTrim2PI(pRobot->theta - Getpt2ptAngle(rbPt, pt2));
			if (anglet2 > PI)
				anglet2 -= 2 * PI;
			if (fabs(anglet1) > fabs(anglet2))
				anglet3 = fabs(anglet2);
			else
				anglet3 = fabs(anglet1);
			
			//
			if (anglet3 < anglelimit || (anglet1 * anglet2 < 0))//xue
			{
				if (anglek > 0) {
					if (anglek < PI / 2)
						pSpeed->LeftValue = (maxspeed - 50) * (PI / 2 - fabs(anglek)) / PI * 2 + 50;
					else
						pSpeed->LeftValue = 50;
					
					pSpeed->RightValue = pSpeed->LeftValue * fabs(cos(anglek)) * kk;
					
				} else {
					
					if (anglek < PI / 2)
						pSpeed->RightValue = (maxspeed - 50) * (PI / 2 - fabs(anglek)) / PI * 2 + 50;
					else
						pSpeed->RightValue = 50;
					//pSpeed->RightValue = maxspeed;
					pSpeed->LeftValue = pSpeed->RightValue * fabs(cos(anglek)) * kk;
					
				}
			}
		}
			//��������
		else if (angle3 > PI - PI * 0.2 && angle3 < PI + PI * 0.2) {
			double anglek;
			anglek = cn_AngleTrim2PI(pRobot->theta + PI - anglerb2target);
			if (anglek > PI)
				anglek -= 2 * PI;
			//xue
			anglet1 = cn_AngleTrim2PI(pRobot->theta - Getpt2ptAngle(rbPt, pt1) + PI);
			if (anglet1 > PI)
				anglet1 -= 2 * PI;
			anglet2 = cn_AngleTrim2PI(pRobot->theta - Getpt2ptAngle(rbPt, pt2) + PI);
			if (anglet2 > PI)
				anglet2 -= 2 * PI;
			if (fabs(anglet1) > fabs(anglet2))
				anglet3 = fabs(anglet2);
			else
				anglet3 = fabs(anglet1);
			
			//
			if (anglet3 < anglelimit || (anglet1 * anglet2 < 0)) {
				if (anglek > 0) {
					//	pSpeed->RightValue = -maxspeed;
					if (anglek < PI / 2)
						pSpeed->RightValue = (maxspeed - 50) * (PI / 2 - fabs(anglek)) / PI * 2 + 50;
					else
						pSpeed->RightValue = 50;
					pSpeed->RightValue = -pSpeed->RightValue;
					
					pSpeed->LeftValue = pSpeed->RightValue * fabs(cos(anglek)) * kk;
				} else {
					if (anglek < PI / 2)
						pSpeed->LeftValue = (maxspeed - 50) * (PI / 2 - fabs(anglek)) / PI * 2 + 50;
					else
						pSpeed->LeftValue = 50;
					pSpeed->LeftValue = -pSpeed->LeftValue;
					
					//	pSpeed->LeftValue = -maxspeed;
					pSpeed->RightValue = pSpeed->LeftValue * fabs(cos(anglek)) * kk;
					
				}
			}
		}
	}
	
	//�Ƕ��ر�õĴ���
	//��������
	if ((angle3 < PI * 0.2 && angle3 >= 0 || angle3 > PI * 1.8 && angle3 < 2 * PI) && fabs(angle1) < PI * 0.45 &&
	    distE <= maxe && (angle2 >= 0 && angle2 <= PI * 0.45 || angle2 >= PI * 1.55 && angle2 <= 2 * PI)) {
		double temp;
		temp = pRobot->y + (PITCH_LENGTH - pRobot->x) * tan(angle2);
		if (temp >= PITCH_WIDTH / 2 - GOALWIDTH / 2 + 1 && temp <= PITCH_WIDTH / 2 + GOALWIDTH / 2 - 1)
			pSpeed->LeftValue = pSpeed->RightValue = maxspeed;
	}
		//��������
	else if ((angle3 > PI - PI * 0.2 && angle3 < PI + PI * 0.2) && fabs(angle1) < PI * 0.45 && distE <= maxe &&
	         angle2 >= PI * 0.55 && angle2 <= PI * 1.45) {
		double temp;
		temp = pRobot->y + (PITCH_LENGTH - pRobot->x) * tan(angle2);
		if (temp >= PITCH_WIDTH / 2 - GOALWIDTH / 2 + 1 && temp <= PITCH_WIDTH / 2 + GOALWIDTH / 2 - 1)
			pSpeed->LeftValue = pSpeed->RightValue = -maxspeed;
	}
	return 1;
	
	
}


double CDecisionMakingx::GetCrossPtWithGLLineli(double x) {
	double target_y = -(ballCharacter.Formu.a * (x + 6) + ballCharacter.Formu.c) / ballCharacter.Formu.b;
	return target_y;
}


int
CDecisionMakingx::ToPositionPDGoalli(dbROBOTPOSTURE *pROBOTPOSTURE, dbPOINT Target, double startspeed, double endspeed,
                                     dbLRWheelVelocity *pLRWheelVelocity) {
	int clock_sign, move_sign;
	double theta, theta_e1;//e1Ϊ��ǰ�Ƕ����
	static double theta_e2 = 0;//e2Ϊ��һ���ڽǶ����
	double dx, dy, dx1, dy1, distance;
	double same_speed;
	
	
	//����任����С������Ϊԭ�㣬С������Ϊy��
	dx1 = Target.x - pROBOTPOSTURE->x;
	dy1 = Target.y - pROBOTPOSTURE->y;
	dx = dx1 * cos(pROBOTPOSTURE->theta - PI / 2) + dy1 * sin(pROBOTPOSTURE->theta - PI / 2);
	dy = -dx1 * sin(pROBOTPOSTURE->theta - PI / 2) + dy1 * cos(pROBOTPOSTURE->theta - PI / 2);
	distance = sqrt(dx * dx + dy * dy);
	if (distance > m_MoveParameter.max_distanceG)
		same_speed = startspeed;
	else
		same_speed = distance / m_MoveParameter.max_distanceG * startspeed;
	if (same_speed < endspeed) same_speed = endspeed;
	theta = atan2(dy, dx);
	if (fabs(fabs(theta) - PI / 2) > m_AngleParameter.MaxAngle) {
		TurnToPointPDli(pROBOTPOSTURE, Target, NOCLOCK, pLRWheelVelocity);
		pLRWheelVelocity->LeftValue /= 3;
		pLRWheelVelocity->RightValue /= 3;
		return 0;
	}
	theta = cn_AngleTrim2PI(theta);
	if (theta <= PI / 2)//��һ����
	{
		move_sign = 1;
		clock_sign = 1;
		theta_e1 = PI / 2 - theta;
	} else if (theta <= PI)//�ڶ�����
	{
		move_sign = 1;
		clock_sign = -1;
		theta_e1 = theta - PI / 2;
	} else if (theta <= 3 * PI / 2)//��������
	{
		move_sign = -1;
		clock_sign = 1;
		theta_e1 = 3 * PI / 2 - theta;
	} else//��������
	{
		move_sign = -1;
		clock_sign = -1;
		theta_e1 = theta - 3 * PI / 2;
	}
	if (theta_e1 > 30.0 / 180.0 * PI)
		same_speed = same_speed * (1 - theta_e1 / (PI / 2));
	double kp, kd;
	kp = m_MoveParameter.kp4pospdG;
	kd = m_MoveParameter.kd4pospdG;
	if (distance < 20 && fabs(theta_e1) < PI / 6) {
		kp *= 0.4;
		kd *= 0.4;
	}
	pLRWheelVelocity->LeftValue = same_speed * move_sign + clock_sign * (kp * theta_e1 + kd * (theta_e1 - theta_e2));
	pLRWheelVelocity->RightValue = same_speed * move_sign - clock_sign * (kp * theta_e1 + kd * (theta_e1 - theta_e2));
	
	//���汾���ڽǶ�����һ������΢����
	theta_e2 = theta_e1;
	return 0;
}

int PointToPointDirectionAngle(dbPOINT Point1, dbPOINT Point2, double *pAngle)
//�������������ӵ�point1����point2�ķ���ǣ��ɹ�����1�����򣬷���0
//��ã�0��2PI)֮��Ļ��ȷ���Ǵ���Angle,
{
	double x, y;
	x = Point2.x - Point1.x;
	y = Point2.y - Point1.y;
	if (x == 0 && y == 0)
		return 0;
	*pAngle = atan2(y, x);
	if (*pAngle < 0)
		*pAngle += 2 * PI;
	return 1;
}

int CDecisionMakingx::TurnToPointPDli(dbROBOTPOSTURE *pRobot, dbPOINT Point, int clock, dbLRWheelVelocity *pSpeed)
//����������ʹС������ת��ָ����
//RobotΪС��λ����Ϣ��PointΪת��ĵ㣬pSpeedΪ���ص���������
//Kp��KdΪ������΢�ֵ��ڲ���
{
	double Angle;
	int Result;
	dbPOINT Point1;
	Point1.x = pRobot->x;
	Point1.y = pRobot->y;
	Result = PointToPointDirectionAngle(Point1, Point, &Angle);
	if (Result == 0)
		return 0;
	Result = TurnToAnglePDli(pRobot, Angle, clock, pSpeed);
	return (Result);
}


int CDecisionMakingx::Move2Ptli(dbROBOTPOSTURE *pROBOTPOSTURE, dbPOINT Target, double speed,
                                dbLRWheelVelocity *pLRWheelVelocity) {
	double theta, temp_theta;
	double dx, dy, dx1, dy1;
	double k1 = 1.0;
	double distance;
	
	//����任����ƽ�ƺ�ת�ǣ���С������Ϊԭ�㣬С������Ϊy��
	dx1 = Target.x - pROBOTPOSTURE->x;
	dy1 = Target.y - pROBOTPOSTURE->y;
	dx = dx1 * cos(pROBOTPOSTURE->theta - PI / 2) + dy1 * sin(pROBOTPOSTURE->theta - PI / 2);
	dy = -dx1 * sin(pROBOTPOSTURE->theta - PI / 2) + dy1 * cos(pROBOTPOSTURE->theta - PI / 2);
	
	distance = sqrt(dx * dx + dy * dy);
	theta = atan2(dy, dx);
	
	if (fabs(dx) < 1) {
		if (dy > 0) {
			pLRWheelVelocity->LeftValue = k1 * speed;
			pLRWheelVelocity->RightValue = k1 * speed;
		} else {
			pLRWheelVelocity->LeftValue = -k1 * speed;
			pLRWheelVelocity->RightValue = -k1 * speed;
		}
	} else if (dx > 1 && dy >= 0)//��һ����
	{
		temp_theta = PI / 2 - theta;
		if (temp_theta < PI / 7) temp_theta *= 2;
		pLRWheelVelocity->LeftValue = k1 * speed;
		pLRWheelVelocity->RightValue = cos(temp_theta) * k1 * speed;
	} else if (dx < -1 && dy >= 0)//�ڶ�����
	{
		temp_theta = theta - PI / 2;
		if (fabs(temp_theta) < PI / 7) temp_theta *= 2;
		pLRWheelVelocity->LeftValue = cos(temp_theta) * k1 * speed;
		pLRWheelVelocity->RightValue = k1 * speed;
	} else if (dx < -1 && dy < 0)//��������
	{
		temp_theta = theta + PI / 2;
		if (fabs(temp_theta) < PI / 7) temp_theta *= 2;
		pLRWheelVelocity->LeftValue = -cos(temp_theta) * k1 * speed;
		pLRWheelVelocity->RightValue = -k1 * speed;
	} else//��������
	{
		temp_theta = theta + PI / 2;
		if (fabs(temp_theta) < PI / 7) temp_theta *= 2;
		pLRWheelVelocity->LeftValue = -k1 * speed;
		pLRWheelVelocity->RightValue = -cos(temp_theta) * k1 * speed;
	}
	return 0;
}


int
CDecisionMakingx::GoalieAction3(dbROBOTPOSTURE *pRobotInford, int x, BallInformation ball, dbLRWheelVelocity *pSpeed,
                                int nRobot) {
	ball.x = m_posBall.GetX();
	ball.y = m_posBall.GetY();
	VecPosition posPlayer = VecPosition(pRobotInford->x, pRobotInford->y);
	VecPosition posBall = VecPosition(ball.x, ball.y);
	int Error_y = 2;                        //block goal area up and down corner
	int xError = 3;                        // homePt.x+xError and homePt.x-xError;
	int gtError = 2, glError = 2;            // gate corner and goal corner +-
	dbPOINT target;
	dbPOINT homePt;
	double dx, dy;
	int goToken = 0;    // go first then turn;
	BOOL IsGate = 0;
	
	
	int sign = 0;
	if (pRobotInford->x <= 0) {
		int uiru = 0;
	}
	homePt.x = x;
	homePt.y = CENTER_Y;
	target = homePt;
	pRobotInford->theta = cn_AngleTrim2PI(pRobotInford->theta);
	if (ball.x < 10)//110
	{
		int lk = 0;
	}
	
	if (ball.x > CENTER_X + 20)//110
	{
		dx = pRobotInford->x - homePt.x;
		dy = pRobotInford->y - homePt.y;
		if (dx * dx + dy * dy < 2 * 2)
			if (pRobotInford->theta < PI)
				TurnToAnglePDli(pRobotInford, PI / 2, NOCLOCK, pSpeed);
			else
				TurnToAnglePDli(pRobotInford, 3 * PI / 2, NOCLOCK, pSpeed);
		else
			ToPositionPDli(pRobotInford, homePt, m_MoveParameter.V_max, 0.0, pSpeed);
	} else {
		if (oldBallPt[0].x > oldBallPt[3].x && oldBallPt[3].x > oldBallPt[6].x)                 // GG new add�����ҷ�����
		{
			target.y = GetCrossPtWithGLLineli(x);             //update oldball previous
		} else
			target.y = ball.y;
		if (target.y < GATE_UP_LINE && target.y > GATE_DN_LINE)//������С����
			IsGate = 1;
		if (ball.x < 10)//x�������źܽ���Ŀ����ڽ�����2���״�
		{
			if (IsGate) {
				if (ball.x < 5) goToken = 0;//ToPositionNew
				else goToken = 1;//Move use max_speed
			} else {
				if (ball.y > 110 && ball.y < 130) {
					target.y = target.y = ball.y + 50;
					goToken = 1;
					sign = 1;//Move use max_speed
				} else if (ball.y < 70 && ball.y > 50) {
					target.y = ball.y - 50;
					goToken = 1;
					sign = 1;//Move use max_speed
				} else if (ball.y < 50) {
					target.y = GOALS_DN_LINE + glError;
					goToken = 2;//ToPositonP:min speed is 20
				} else if (target.y > 130) {
					target.y = GOALS_UP_LINE - glError;
					goToken = 2;
				}
			}
		} else if (ball.x < homePt.x + 20)//����<25����.������������ױ�б�䣬��Ӧ������
		{
			if (IsGate)
				if (oldBallPt[6].x < oldBallPt[4].x)goToken = 1;//move use max_speed
				else goToken = 2;//ToPositonP:min speed is 20
			else {
				if (target.y < GOALS_DN_LINE)
					if (oldBallPt[4].x > oldBallPt[6].x && oldBallPt[4].y < oldBallPt[6].y)
						target.y = GATE_DN_LINE + gtError;
					else
						target.y = GATE_DN_LINE - gtError;
				else if (target.y > GOALS_UP_LINE)
					if (oldBallPt[4].x > oldBallPt[6].x && oldBallPt[4].y > oldBallPt[6].y)
						target.y = GATE_UP_LINE - gtError;
					else
						target.y = GATE_UP_LINE + gtError;
				goToken = 3;//ToPositonP:min speed is 15
			}
		} else if (ball.x < homePt.x + 40)//����<45����
		{
			if (IsGate) goToken = 3;//ToPositonP:min speed is 15
			else {
				if (target.y < GATE_DN_LINE)
					target.y = GATE_DN_LINE;
				else if (target.y > GATE_UP_LINE)
					target.y = GATE_UP_LINE;
				goToken = 4;//ToPositonP:min speed is 10
			}
		} else if (ball.x < homePt.x + 60)//����<65����
		{
			if (target.y < GATE_DN_LINE)
				target.y = GATE_DN_LINE + gtError;
			else if (target.y > GATE_UP_LINE)
				target.y = GATE_UP_LINE - gtError;
			goToken = 4;//ToPositonP:min speed is 10
		} else if (ball.x < homePt.x + 70)//����<75����
		{
			if (target.y < GATE_DN_LINE)
				target.y = CENTER_Y - goal_y_widthS / 2;
			else if (target.y > GATE_UP_LINE)
				target.y = CENTER_Y + goal_y_widthS / 2;
			goToken = 5;//ToPositonP:min speed is 5
		} else //����>75����
		{
			if (target.y < GATE_DN_LINE)
				target.y = CENTER_Y - goal_y_widthS / 4;
			else if (target.y > GATE_UP_LINE)
				target.y = CENTER_Y + goal_y_widthS / 4;
			goToken = 5;//ToPositonP:min speed is 5
		}
		//sign=1;
		if (sign == 1) {
			/*
			if(target.y > GOALS_UP_LINE+2)		//target.y =CENTER_Y+3;//
				target.y = GOALS_UP_LINE+3;
			else if(target.y < GOALS_DN_LINE-2)	//target.y =CENTER_Y-3;//
			target.y = GOALS_DN_LINE-3;
			*/
		}
		dx = pRobotInford->x - target.x;
		dy = pRobotInford->y - target.y;
		
		switch (goToken) {
			case 0:
				dbPOINT tempPt;
				tempPt.x = 6;
				if (ball.y < pRobotInford->y)
					tempPt.y = 10;
				else
					tempPt.y = 170;
				
				ToPositionNewli(pRobotInford, target, tempPt, 120, 0, pSpeed);
				break;
			case 1:
				if (dx * dx + dy * dy < 4 * 4)
//			{	VecPosition target2(target.x,target.y);
//			ToPositionPDGoal(nRobot,target2,m_MoveParameter.V_max,5.0);}
					ToPositionPDGoalli(pRobotInford, target, m_MoveParameter.V_max, 5.0, pSpeed);
				else
					Move2Ptli(pRobotInford, target, m_MoveParameter.V_max, pSpeed);
				break;
			case 2:
				if (dx * dx + dy * dy < 4 * 4)
					//{VecPosition target2(target.x,target.y);
					//ToPositionPDGoal(nRobot,target2,m_MoveParameter.V_max,5.0);}
					ToPositionPDGoalli(pRobotInford, target, m_MoveParameter.V_max, 5.0, pSpeed);
				else
					//{VecPosition target2(target.x,target.y);
					//ToPositionPDGoal(nRobot,target2,m_MoveParameter.V_max,5.0);}
					ToPositionPDGoalli(pRobotInford, target, m_MoveParameter.V_max, 20.0, pSpeed);
				break;
			case 3:
				//{VecPosition target2(target.x,target.y);
				//ToPositionPDGoal(nRobot,target2,m_MoveParameter.V_max,5.0);}
				ToPositionPDGoalli(pRobotInford, target, m_MoveParameter.V_max, 15.0, pSpeed);
				break;
			case 4:
				//{VecPosition target2(target.x,target.y);
				//ToPositionPDGoal(nRobot,target2,m_MoveParameter.V_max,5.0);}
				ToPositionPDGoalli(pRobotInford, target, m_MoveParameter.V_max, 10.0, pSpeed);
				break;
			case 5:
				//{VecPosition target2(target.x,target.y);
				//ToPositionPDGoal(nRobot,target2,m_MoveParameter.V_max,5.0);}
				ToPositionPDGoalli(pRobotInford, target, m_MoveParameter.V_max, 5.0, pSpeed);
				break;
			default:
				break;
		}
		if (!(goToken == 0 || goToken == 1)) {
			if (dx * dx + dy * dy < 2 * 2)
				if (pRobotInford->theta < PI)
					TurnToAnglePDli(pRobotInford, PI / 2, NOCLOCK, pSpeed);
				else
					TurnToAnglePDli(pRobotInford, 3 * PI / 2, NOCLOCK, pSpeed);
		}
	}
	if (pRobotInford->x >= 30) {
		int yu = 12;
	}
/*	FILE * fp1;
	fp1=fopen("d:\\iori1.txt","at");
	fprintf(fp1," %f",target.x);
	fprintf(fp1," %f",target.y);
	fprintf(fp1,"    ");
	fprintf(fp1," %f",pRobotInford->x);
	fprintf(fp1," %f",pRobotInford->y);
	fputs("\n",fp1);
	fclose(fp1);
	*/
	return 1;
}

int
CDecisionMakingx::GoalieAction4(dbROBOTPOSTURE *pRobotInford, int x, BallInformation ball, dbLRWheelVelocity *pSpeed,
                                int nRobot) {
	ball.x = m_posBall.GetX();
	ball.y = m_posBall.GetY();
	VecPosition posPlayer = VecPosition(pRobotInford->x, pRobotInford->y);
	VecPosition posBall = VecPosition(ball.x, ball.y);
	int Error_y = 2;                        //block goal area up and down corner
	int xError = 3;                        // 	homePt.x+xError and homePt.x-xError;
	int gtError = 2, glError = 2;            // gate corner and goal corner +-
	dbPOINT target;
	dbPOINT homePt;
	double dx, dy;
	int goToken = 0;    // go first then turn;
	BOOL IsGate = 0;
	int sign = 0;
	
	homePt.x = x;
	homePt.y = CENTER_Y;
	target = homePt;
	pRobotInford->theta = cn_AngleTrim2PI(pRobotInford->theta);
	
	if ((dmDEG.DEStartMode = PenaltyKick) && (dmDEG.DEStartState == 1)) {
		int uii = 1;
	}
	if (ball.x > CENTER_X + 20) {
		dx = pRobotInford->x - homePt.x;
		dy = pRobotInford->y - homePt.y;
		if (dx * dx + dy * dy < 2 * 2)
			if (pRobotInford->theta < PI)
				TurnToAnglePDli(pRobotInford, PI / 2, NOCLOCK, pSpeed);
			else
				TurnToAnglePDli(pRobotInford, 3 * PI / 2, NOCLOCK, pSpeed);
		else
			ToPositionPDli(pRobotInford, homePt, m_MoveParameter.V_max, 0.0, pSpeed);
	} else {
		if (oldBallPt[0].x > oldBallPt[3].x && oldBallPt[3].x > oldBallPt[6].x)
			
			// GG new add
		{
			target.y = GetCrossPtWithGLLineli(x);             //update oldball previous
		} else
			target.y = ball.y;
		if (target.y < GATE_UP_LINE && target.y > GATE_DN_LINE)//������С����
			IsGate = 1;
		if (ball.x < 10)//x�������źܽ���Ŀ����ڽ�����2���״�
		{
			if (IsGate) {
				if (ball.x < 5) goToken = 0;//ToPositionNewli
				else goToken = 1;//Move use max_speed
			} else {
				if (ball.y > 110 && ball.y < 130) {
					target.y = ball.y + 50;
					goToken = 1;
					sign = 1;//Move use max_speed
				} else if (ball.y < 70 && ball.y > 50) {
					target.y = ball.y - 50;
					goToken = 1;
					sign = 1;//Move use max_speed
				} else if (ball.y < 50) {
					target.y = GOALS_DN_LINE + glError;
					goToken = 2;//ToPositonP:min speed is 20
				} else if (target.y > 130) {
					target.y = GOALS_UP_LINE - glError;
					goToken = 2;
				}
			}
		} else if (ball.x < homePt.x + 20)//����<25����
		{
			if (IsGate)
				if (oldBallPt[6].x < oldBallPt[4].x)goToken = 1;//move use max_speed
				else goToken = 2;//ToPositonP:min speed is 20
			else {
				if (target.y < GOALS_DN_LINE)
					if (oldBallPt[4].x > oldBallPt[6].x && oldBallPt[4].y < oldBallPt[6].y)
						target.y = GATE_DN_LINE + gtError;
					else
						target.y = GATE_DN_LINE - gtError;
				else if (target.y > GOALS_UP_LINE)
					if (oldBallPt[4].x > oldBallPt[6].x && oldBallPt[4].y > oldBallPt[6].y)
						target.y = GATE_UP_LINE - gtError;
					else
						target.y = GATE_UP_LINE + gtError;
				goToken = 3;//ToPositonP:min speed is 15
			}
		} else if (ball.x < homePt.x + 40)//����<45����
		{
			if (IsGate) goToken = 3;//ToPositonP:min speed is 15
			else {
/*				if(ball.x < homePt.x + 80)
				{
					if ((ball.y >=GATE_DN_LINE)&&(ball.y <=GOAL_DN_LINE))
				{
					target.y = CENTER_Y - goal_y_widthS ;
				}
				if ((ball.y <=GATE_UP_LINE)&&(ball.y >=GOAL_UP_LINE))
				{
					target.y = CENTER_Y + goal_y_widthS ;
				}
				}

  */
				if (target.y < GATE_DN_LINE)
					target.y = GATE_DN_LINE;
				else if (target.y > GATE_UP_LINE)
					target.y = GATE_UP_LINE;
				goToken = 4;//ToPositonP:min speed is 10
			}
		} else if (ball.x < homePt.x + 60)//����<65����
		{
			if (target.y < GATE_DN_LINE)
				target.y = GATE_DN_LINE + gtError;
			else if (target.y > GATE_UP_LINE)
				target.y = GATE_UP_LINE - gtError;
			goToken = 4;//ToPositonP:min speed is 10
		} else if (ball.x < homePt.x + 70)//����<75����
		{
			if (target.y < GATE_DN_LINE)
				target.y = CENTER_Y - goal_y_widthS / 2;
			else if (target.y > GATE_UP_LINE)
				target.y = CENTER_Y + goal_y_widthS / 2;
			goToken = 5;//ToPositonP:min speed is 5
		} else //����>75����
		{
			if (target.y < GATE_DN_LINE)
				target.y = CENTER_Y - goal_y_widthS / 4;
			else if (target.y > GATE_UP_LINE)
				target.y = CENTER_Y + goal_y_widthS / 4;
			goToken = 5;//ToPositonP:min speed is 5
		}
		if (sign != 1) {
//			if(target.y > GOALS_UP_LINE+2)		target.y = GOALS_UP_LINE+3;
//			else if(target.y < GOALS_DN_LINE-2)	target.y = GOALS_DN_LINE-3;
		}
		dx = pRobotInford->x - target.x;
		dy = pRobotInford->y - target.y;
		switch (goToken) {
			case 0:
				dbPOINT tempPt;
				tempPt.x = 6;
				if (ball.y < pRobotInford->y)
					tempPt.y = 10;
				else
					tempPt.y = 170;
				ToPositionNewli(pRobotInford, target, tempPt, 120, 0, pSpeed);
				break;
			case 1:
				if (dx * dx + dy * dy < 4 * 4)
					ToPositionPDGoalli(pRobotInford, target, m_MoveParameter.V_max, 5.0, pSpeed);
				else
					Move2Ptli(pRobotInford, target, m_MoveParameter.V_max, pSpeed);
				break;
			case 2:
				if (dx * dx + dy * dy < 4 * 4)
					ToPositionPDGoalli(pRobotInford, target, m_MoveParameter.V_max, 5.0, pSpeed);
				else
					ToPositionPDGoalli(pRobotInford, target, m_MoveParameter.V_max, 20.0, pSpeed);
				break;
			case 3:
				ToPositionPDGoalli(pRobotInford, target, m_MoveParameter.V_max, 15.0, pSpeed);
				break;
			case 4:
				ToPositionPDGoalli(pRobotInford, target, m_MoveParameter.V_max, 10.0, pSpeed);
				break;
			case 5:
				ToPositionPDGoalli(pRobotInford, target, m_MoveParameter.V_max, 5.0, pSpeed);
				break;
			default:
				break;
		}
		if (!(goToken == 0 || goToken == 1)) {
			if (dx * dx + dy * dy < 2 * 2)
				if (pRobotInford->theta < PI)
					TurnToAnglePDli(pRobotInford, PI / 2, NOCLOCK, pSpeed);
				else
					TurnToAnglePDli(pRobotInford, 3 * PI / 2, NOCLOCK, pSpeed);
		}
	}

/*	FILE * fp1;
	fp1=fopen("d:\\iori1.txt","at");
	fprintf(fp1," %f",target.x);
	fprintf(fp1," %f",target.y);
	fprintf(fp1,"    ");
	fprintf(fp1," %f",pRobotInford->x);
	fprintf(fp1," %f",pRobotInford->y);
	fputs("\n",fp1);
	fclose(fp1);
*/
	return 1;
}


double CDecisionMakingx::AdjustTarget() {
	double target_y;
	float RobotWidth = 8;
	//int    PosiErr=5;//�����������Ҫ����
	int PosiErr = 5;//�����������Ҫ����
	int detaX;
	double slope;
	slope = (GOAL_WIDTH - PosiErr - RobotWidth / 2) / (G_OFFSET + RobotWidth / 2);
	
	if (ball.y < PITCH_WIDTH / 2 + GOAL_WIDTH / 2 - slope * m_posBall.GetX())
		target_y = PITCH_WIDTH / 2 - GOAL_WIDTH / 2 - PosiErr;//�Ƿ�������
	else if (ball.y > PITCH_WIDTH / 2 - GOAL_WIDTH / 2 + slope * m_posBall.GetX())
		target_y = PITCH_WIDTH / 2 + GOAL_WIDTH / 2 + PosiErr;
	else {
		detaX = 0;
		target_y = 2 - (ballCharacter.Formu.a * (G_OFFSET + RobotWidth / 2 + detaX) + ballCharacter.Formu.c) /
		               ballCharacter.Formu.b;
	}
	if (m_posBall.GetX() <= 18.5) {
		target_y = m_posBall.GetY();
	}
	return target_y;
}


/************************************************************************************
������	��GetCrossPtWithGLLine()
�������ڣ�19/4/2001
����	��
����	��Ԥ������˶��������ҷ����ŵĽ���
************************************************************************************/
double CDecisionMakingx::GetCrossPtWithGLLine() {
	double detaX, target_y;

#ifdef SimuroSot5
	detaX = 0;
#else
	/*if(m_posBall.GetX() < 30)
	{
		return m_posBall.GetY();
	}
	*/
	// ֱ�߱Ƚ϶���y�����ϱ仯��Χ��
	if (fabs(ballCharacter.Formu.a / ballCharacter.Formu.b) > tan(75.0 / 180.0 * PI))
		detaX = 8;
	else if (fabs(ballCharacter.Formu.a / ballCharacter.Formu.b) > tan(50.0 / 180.0 * PI))
		detaX = 6;
	else if (fabs(ballCharacter.Formu.a / ballCharacter.Formu.b) > tan(35.0 / 180.0 * PI))
		detaX = 4;
	else if (fabs(ballCharacter.Formu.a / ballCharacter.Formu.b) > tan(20.0 / 180.0 * PI))
		detaX = 2;
	else
		detaX = 0;

#endif
	
	// �õ�x=G_OFFSET+detaXʱ��y����
	target_y = -(ballCharacter.Formu.a * (G_OFFSET + ROBOT_LENGTH + detaX) + ballCharacter.Formu.c) /
	           ballCharacter.Formu.b;
	
	return target_y;
}


/////////////////// Goalie End ///////////////////////////////////////////////////////


/************************************************************************/
/* ��ʼ��                                                               */
/************************************************************************/
void CDecisionMakingx::Initialize() {
	together = FALSE;
	round = FALSE;
	m_nTimer = 0;
	m_nTimerLastStart = -1;
	bStartUnUsual = TRUE;
	nReset = FALSE;
	
	for (int i = 0; i < BALLNUMPARA; i++) {
		oldBallPt[i].x = 0;
		oldBallPt[i].y = 0;
	}
	for (i = 0; i < ROBOTNUMBER; i++) {
		oldrbV[i].LeftValue = oldrbV[i].RightValue = 0;
		oldRobot[i].x = oldRobot[i].y = oldRobot[i].theta = 0;
	}
	
	for (i = 0; i < 2 * ROBOTNUMBER + 1; i++) {
		Robot[i].x = 0;
		Robot[i].y = 0;
		Robot[i].theta = 0;
	}
	
	for (i = 0; i < 3; i++)
		sym[i] = 0;
	
	for (i = 0; i < 100; i++)
		effecChar[i] = 0;
	
	m_bBallKick = FALSE;
	
	for (i = 0; i < ROBOTNUMBER; i++)
		m_bShootLine[i] = FALSE;
	
	
	m_Front = 0.0;
	m_MoveParameter.V_MAX = 120.0;
	m_MoveParameter.V_max = 100.0;
	m_MoveParameter.max_distance = 45.0;
	m_MoveParameter.kp4pospd = 100;//19.0;
	m_MoveParameter.kd4pospd = 0.50;
	m_MoveParameter.max_distanceG = 25.0;
	m_MoveParameter.kp4pospdG = 50;//20.50;
	m_MoveParameter.kd4pospdG = 1.90;
	
	m_AngleParameter.Kp = 80;//36.0;
	m_AngleParameter.Kd = 5;//7.0;
	m_AngleParameter.AngleError = 0;//5.0/180.0*pi;
	m_AngleParameter.MaxAngleSpeed = 80.0;
	m_AngleParameter.MaxAngle = 70.0 * PI / 180.0;
	m_decisionparamter.StartSW = 1;
	
	//m_decisionparamter.oldball.x = 0;
	//decisionparamter.oldball.y = 0;
	m_decisionparamter.nReset = FALSE;
	m_decisionparamter.numPara = 7;
	for (i = 0; i < m_decisionparamter.numPara; i++) {
		oldBallPt[i].x = 0;
		oldBallPt[i].y = 0;
	}
	
}

/************************************************************************/
/*  ��ʼ������������                                                  */
/************************************************************************/
void CDecisionMakingx::InitDEG(DEGame DEG) {
	dmDEG = DEG;
}

/************************************************************************/
/* �������                                                             */
/************************************************************************/
void CDecisionMakingx::Startx(RobotInford dmRobot[]) {
//	m_PerfTimer.Start(TRUE);
	
	//���Ӿ����ݴ���������
	for (int i = 0; i < 2 * ROBOTNUMBER + 1; i++) {
		Robot[i].x = dmRobot[i].x;
		Robot[i].y = dmRobot[i].y;
		Robot[i].theta = dmRobot[i].theta;
		Robot[i].theta = VecPosition::NormalizeAngle(Robot[i].theta);
	}
	
	PreProcess();//��ϢԤ����
	
	if (nReset)
		RobotReturn2Pt();
	else
		MiroSot_DecisionMaking();

//	m_PerfTimer.Stop();
//	const double dTime = m_PerfTimer.Elapsedms();
//	SendMsg(3, "Time = %2.2f", dTime);
}

/************************************************************************/
/*  ��ϢԤ������                                                      */
/************************************************************************/
void CDecisionMakingx::PreProcess() {
	m_nTimer++;
	
	// ��ʱ����ʱ
	m_timerBallKick.Step();
	m_timerShootLine.Step();
	m_timerStart.Step();
	
	
	//���Ұ볡ת��
	if (dmDEG.DEGameGround == RightArea) {
		for (int i = 0; i < 2 * ROBOTNUMBER + 1; i++) {
			Robot[i].x = PITCH_LENGTH - Robot[i].x;
			Robot[i].y = PITCH_WIDTH - Robot[i].y;
			Robot[i].theta = Robot[i].theta + PI;
			if (Robot[i].theta > 2 * PI) Robot[i].theta -= 2 * PI;
		}
	}
	
	m_posBall.SetVecPosition(Robot[2 * ROBOTNUMBER].x, Robot[2 * ROBOTNUMBER].y);
	ball = m_posBall.GetdbPOINT();
	
	// ���ˣ����أ�100
	if ((m_posBall.GetX() + 100) < EPS) {
		if (sym[0]) {
			double L = 4;
			if (sym[1]) {
				m_posBall.SetX(Robot[sym[2]].x + L * cos(Robot[sym[2]].theta));
				m_posBall.SetY(Robot[sym[2]].y + L * sin(Robot[sym[2]].theta));
			} else {
				m_posBall.SetX(Robot[sym[2]].x + L * cos(Robot[sym[2]].theta + PI));
				m_posBall.SetY(Robot[sym[2]].y + L * sin(Robot[sym[2]].theta + PI));
			}
		}
	}
	//�����¼
	sym[0] = 0;
	
	if (m_nTimer == 1) {
		for (int i = 0; i < ROBOTNUMBER; i++) {
			oldRobot[i] = Robot[i];
		}
	}
	
	
	//��¼�����Ϣ
	// �����ײ,���¼�����켣
	if (!m_bBallKick && m_posPostBall.GetDistanceTo(m_posBall) > 2) {
		m_bBallKick = TRUE;
		m_timerBallKick.Start(10);
		
		for (int i = 0; i < BALLNUMPARA; i++) {
			oldBallPt[i].x = ball.x;
			oldBallPt[i].y = ball.y;
		}
		
		for (i = 0; i < ROBOTNUMBER; i++)
			m_bShootLine[i] = FALSE;
	} else {
		for (int i = 0; i < BALLNUMPARA - 1; i++) {
			oldBallPt[i].x = oldBallPt[i + 1].x;
			oldBallPt[i].y = oldBallPt[i + 1].y;
		}
		oldBallPt[BALLNUMPARA - 1].x = ball.x;
		oldBallPt[BALLNUMPARA - 1].y = ball.y;
	}
	
	
	Forcastball(oldBallPt, &ballCharacter);
	m_posPostBall = PredictBall(1);
	
	// ��ֹ��������
	for (int i = 0; i < ROBOTNUMBER; i++) {
		if (fabs(Robot[i].x + 100) < EPS && fabs(Robot[i].y + 100) < EPS) {
			Robot[i] = oldRobot[i];
		}
	}
	
	// ���������, ��ʼ���������ٶ�
	for (i = 0; i < ROBOTNUMBER; i++) {
		Robot[i].speedv = sqrt((Robot[i].x - oldRobot[i].x) * (Robot[i].x - oldRobot[i].x)
		                       + (Robot[i].y - oldRobot[i].y) * (Robot[i].y - oldRobot[i].y));
		Robot[i].speedw = Robot[i].theta - oldRobot[i].theta;
		
		oldRobot[i] = Robot[i];
		rbV[i].LeftValue = rbV[i].RightValue = 0;
	}
	
	// Others
	for (i = 0; i < ROBOTNUMBER; i++)
		m_bRobotShoot[i] = FALSE;
}

/************************************************************************/
/*  MIROSOT�������                                                     */
/************************************************************************/
void CDecisionMakingx::MiroSot_DecisionMaking() {
	//LogData();
	
	int areaNo = GetAreaNo(m_posBall);//�õ��������
	SendMsg(2, "%d", areaNo);
	if (areaNo == 0)// || areaNo != 1)
		return;
	
	TaskDecompose(areaNo);//������ֽ����������
//#define TEST1
#ifdef TEST1
	Test();
#else
	FormInterpret(dmformNo);//���ν���
	CharAllot();//��ɫ����
	
	RobotManager();//�����˹���
	
	ActProcess();//����ִ��
	
	
	//SendMsg(0, "%d : %d, %d, %d, %d", 		areaNo, currentResult[0], currentResult[1], currentResult[2], currentResult[3]);
#endif


#ifndef TEST
/*
	VecPosition posTarget;
	double angleRobot;
	int action2;
	for (int i=0; i<ROBOTNUMBER-1; i++)
	{
		BOOL bHave = m_formation.GetStrategyPosition(currentResult[i], m_posBall,
						posTarget, angleRobot, action2);
		if (bHave)
		{
			SendMsg(0, "\t %d : (%2.2f, %2.2f), %2.2f", i, posTarget.GetX(), posTarget.GetY(), angleRobot);
		}
	}
*/
#endif
	
	return;
}

/************************************************************************/
/*  �������λ�õķ������õ��������                                    */
/************************************************************************/
int CDecisionMakingx::GetAreaNo(VecPosition posBall) {
	//posBall.SetX(x);
	//posBall.SetY(y);
	
	BOOL bButtonSide;
	if (posBall.IsTopOf(PITCH_WIDTH / 2)) {
		posBall.SetY(PITCH_WIDTH - posBall.GetY());
		bButtonSide = FALSE;
	} else {
		bButtonSide = TRUE;
	}
	
	int nAreaNo = 0;
	
	Line line1, line2, line3, line4;
	line1 = Line::MakeLineFromPositionAndAngle(
			VecPosition(LINE1, GOAL_AREA_LENGTH), -PI / 4);
	line2 = Line::MakeLineFromPositionAndAngle(
			VecPosition(GOAL_AREA_LENGTH, PITCH_WIDTH / 2 - GOAL_AREA_WIDTH / 2), -PI / 4);
	line3 = Line::MakeLineFromPositionAndAngle(
			VecPosition(PITCH_LENGTH - LINE1, GOAL_AREA_LENGTH), PI / 4);
	line4 = Line::MakeLineFromPositionAndAngle(
			VecPosition(PITCH_LENGTH - GOAL_AREA_LENGTH, PITCH_WIDTH / 2 - GOAL_AREA_WIDTH / 2), PI / 3);
	
	if (posBall.GetX() < -GOAL_DEPTH) {
		nAreaNo = 0;
	} else if (posBall.GetX() < GOAL_AREA_LENGTH) {
		if (posBall.GetY() > PITCH_WIDTH / 2 - GOAL_AREA_WIDTH / 2) {
			nAreaNo = 4;
			if (posBall.GetX() <= 10) {
				nAreaNo = 30;
			}
			
		} else if (posBall.GetY() > PITCH_WIDTH / 2 - PENALTY_AREA_WIDTH / 2)
			nAreaNo = 3;
		else if (posBall.GetY() > LINE1)
			nAreaNo = 2;
		else
			nAreaNo = 1;
	} else if (posBall.GetX() < PENALTY_AREA_LENGTH) {
		if (line2.GetYGivenX(posBall.GetX()) < posBall.GetY()) {
			nAreaNo = 7;
			if (posBall.GetX() <= (PENALTY_AREA_LENGTH - GOAL_AREA_LENGTH) / 2 + GOAL_AREA_LENGTH)
				nAreaNo = 28;
		} else if (line1.GetYGivenX(posBall.GetX()) < posBall.GetY()
		           && posBall.GetY() > GOAL_AREA_LENGTH) {
			nAreaNo = 6;
			//		if (posBall.GetY()>110)
			//			nAreaNo=29;
		} else if (posBall.GetX() < LINE1)
			nAreaNo = 1;
		else
			nAreaNo = 5;
	} else if (posBall.GetX() < LINE3) {
		if (line2.GetYGivenX(posBall.GetX()) < posBall.GetY())
			nAreaNo = 8;
		else if (posBall.GetY() < GOAL_AREA_LENGTH)
			nAreaNo = 5;
		else
			nAreaNo = 6;
	} else if (posBall.GetX() < PITCH_LENGTH / 2) {
		if (posBall.GetY() > LINE2)
			nAreaNo = 11;
		else if (posBall.GetY() > GOAL_AREA_LENGTH)
			nAreaNo = 10;
		else
			nAreaNo = 9;
	} else if (posBall.GetX() < line4.GetXGivenY(GOAL_AREA_LENGTH)) {
		if (posBall.GetY() > LINE2)
			nAreaNo = 14;
		else if (posBall.GetY() > GOAL_AREA_LENGTH)
			nAreaNo = 13;
		else
			nAreaNo = 12;
	} else if (posBall.GetX() < PITCH_LENGTH - GOAL_AREA_LENGTH) {
		if (line4.GetYGivenX(posBall.GetX()) < posBall.GetY())
			nAreaNo = 16;
		else if (line3.GetYGivenX(posBall.GetX()) < posBall.GetY()
		         && posBall.GetY() > GOAL_AREA_LENGTH)
			nAreaNo = 15;
		else if (posBall.GetX() < PITCH_LENGTH - LINE1)
			nAreaNo = 12;
		else
			nAreaNo = 17;
	} else if (posBall.GetX() <= PITCH_LENGTH + GOAL_DEPTH) {
		if (posBall.GetY() > PITCH_WIDTH / 2 - GOAL_AREA_WIDTH / 2)
			nAreaNo = 19;
		else if (posBall.GetY() > PITCH_WIDTH / 2 - PENALTY_AREA_WIDTH / 2)
			nAreaNo = 18;
		else if (posBall.GetY() > LINE1)
			nAreaNo = 18;
		else
			nAreaNo = 17;
	} else {
		nAreaNo = 0;
	}

/*#ifdef SimuroSot11
	if (nAreaNo == 11)
	{
		if ( (posBall.GetX() < LINE3 + PITCH_LENGTH/10)
			&& (posBall.GetY() < PITCH_WIDTH/2 - GOAL_AREA_WIDTH/2) )
			nAreaNo = 20;
	}
	if (nAreaNo == 11 || nAreaNo == 14)
	{
		if (posBall.GetY() > PITCH_WIDTH/2 - GOAL_AREA_WIDTH/2)
			nAreaNo = 21;
	}
#endif
	*/

/*	FILE * fp;
	fp=fopen("d:\\iori.txt","at");
	fprintf(fp," %f",posBall.GetX());
	fprintf(fp," %f",posBall.GetY());
	fputs("\n",fp);
	fclose(fp);
	FILE * fp1;
	fp1=fopen("d:\\iori1.txt","at");
	for (int
		i=0; i<ROBOTNUMBER-1; i++)
	{
		fprintf(fp1," %d",nAreaNo);
	}
	fputs("\n",fp1);
	fclose(fp1);


*/
/*	if (tagright==1)
	{
		tagright=1;
		nAreaNo=tagright*50+nAreaNo;
	}
	*/
	return nAreaNo;
}

/************************************************************************/
/* ����ֽ�õ����κ���                                                 */
/************************************************************************/
void CDecisionMakingx::TaskDecompose(int areaNo) {
	dmformNo = areaNo;
	
	//����״̬������
	//���ǿ������˳�����ֽ⺯��
	if (!bStartUnUsual)
		return;
	
	return;
	
	if (dmDEG.DEStartMode == NormalStart)
		m_timerStart.Start(30);
	else
		m_timerStart.Start(30);
	
	if (m_nTimerLastStart = -1)
		m_nTimerLastStart = m_nTimer;
	
	switch (dmDEG.DEStartMode) {
		case NormalStart://����
		{
			if (dmDEG.DEStartState == Attack)
				dmformNo = 30;//�ҷ�
			else
				dmformNo = 31;//�Է�����
		}
			break;
		case PenaltyKick://����
		{
			if (dmDEG.DEStartState == Attack) {
				dmformNo = 32;//�ҷ�
			} else
				dmformNo = 33;//�Է�����
		}
			break;
		case FreeKick://������
		{
			if (dmDEG.DEStartState == Attack)
				dmformNo = 34;//�ҷ�
			else
				dmformNo = 35;//�Է�����
		}
			break;
		case FreeBall://����
			dmformNo = 36;
			break;
		case GoalKick://����
			dmformNo = 37;
			break;
	}
	
}

/************************************************************************/
/* ���ν��ͺ���                                                         */
/************************************************************************/
void CDecisionMakingx::FormInterpret(int formNo) {
	
	//FILE * fp;
	//fp=fopen("d:\\iori.txt","at");
	for (int i = 0; i < ROBOTNUMBER - 1; i++) {
		m_nPriorityForm[i] = i + 1;
		currentForm[i] = m_formation.m_nForm[formNo][i];
		//	fprintf(fp," %d",currentForm[i]);
	}
	//fputs("\n",fp);

//	fclose(fp);
	
	// ����Ա
	currentForm[ROBOTNUMBER - 1] = 12;
	m_nPriorityForm[ROBOTNUMBER - 1] = 0;
}

/************************************************************************/
/*  ��ɫ���ͺ���                                                        */
/************************************************************************/
void CDecisionMakingx::CharInterpret(int nRobot, int charNo, dbLRWheelVelocity *pSpeed) {
//	dbPOINT pt ;
//	double  tempy;
	VecPosition posTarget;
	double angleRobot;
	int nAction2;
	BOOL bButtonSide = TRUE;
	VecPosition posBall = m_posBall;
	if (posBall.GetY() > PITCH_WIDTH / 2) {
		posBall.SetY(PITCH_WIDTH - posBall.GetY());
		bButtonSide = FALSE;
	}
	
	BOOL bHave = m_formation.GetStrategyPosition(charNo, posBall,
	                                             posTarget, angleRobot, nAction2);
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	if (bHave) {
		if (!bButtonSide) {
			posTarget.SetY(PITCH_WIDTH - posTarget.GetY());
			angleRobot = -angleRobot;
		}
		
		Wait(nRobot, posTarget, angleRobot, 100);
		
		SendMsg(0, "%d : Wait at (%2.2f, %2.2f)", nRobot, posTarget.GetX(), posTarget.GetY());
		
		if (nAction2 == 6) {
			m_bRobotShoot[nRobot] = TRUE;
		} else {
			m_bRobotShoot[nRobot] = FALSE;
		}
	} else {
		switch (charNo) {
			case 0:    //��ֹ
			{
				pSpeed->LeftValue = pSpeed->RightValue = 0;
			}
				break;
			
			case 5://��ɫ5��������
			{
				SendMsg(0, "%d BoundPushBall", nRobot);
				BoundPushBall(nRobot);
				m_bRobotShoot[nRobot] = TRUE;
				
				//GoalieAction4(&Robot[nRobot],45,ball,pSpeed,nRobot);
			}
				break;
			
			case 6: {
				SendMsg(0, "%d KickBall", nRobot);
				KickBall(nRobot);
				m_bRobotShoot[nRobot] = TRUE;
			}
				break;
			
			case 11://��ɫ11����
			{
				BallInformation ballli;
				//ballli=new(BallInformation *);
				ballli.x = m_posBall.GetX();
				ballli.y = m_posBall.GetY();
				SendMsg(0, "%d Vect_MidShoot", nRobot);
				Vect_MidShootli(Robot[nRobot], ballli, pSpeed);
				//	Vect_MidShoot(nRobot);
				m_bRobotShoot[nRobot] = TRUE;
			}
				break;
			
			case 12://��ɫ12����
			{
				SendMsg(0, "%d GoalieAction", nRobot);
				//		GoalieAction(nRobot);
				//if (tagright=1)
				//{
				GoalieAction4(&Robot[nRobot], 10, ball, pSpeed, nRobot);
				//}
				//else
//				GoalieAction3(&Robot[nRobot],10,ball,pSpeed,nRobot);
				
				//GoalieAction2(&Robot[robotNo],3, ball,rbV);
			}
				break;
			
			case 22://����
			{
				SendMsg(0, "%d Mark", nRobot);
				Mark(nRobot);
			}
				break;
			
			case 21://һ��һ��������
			{
				GoalieAction3(&Robot[nRobot], 45, ball, pSpeed, nRobot);
			}
				break;

/*		case 24://һ��һ��������
			{
				VecPosition pos;
				pos.SetX(m_posBall.GetX()-20);
				pos.SetY( (PITCH_LENGTH - pos.GetX())/2. + PITCH_WIDTH/2-15);
				
				VecPosition posGoal(PITCH_LENGTH + 3, PITCH_WIDTH/2-15);
				
				VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
				
				if((pos - posRobot).GetMagnitude() > 3)
					ToPositionPD(&Robot[nRobot], pos,70,rbV);
				else
					TurnToPointPD(&Robot[nRobot], posGoal,0,rbV);
			}
			break;
*/
			case 39://����ش�
			{
				if (m_nTimer - m_nTimerLastStart < STARTTIME) {
					pSpeed->LeftValue = pSpeed->RightValue = 8;
				} else {
					pSpeed->LeftValue = -80;
					pSpeed->RightValue = -50;
				}
			}
				break;
			case 40://��������
			{
				if (m_nTimer - m_nTimerLastStart < STARTTIME) {
					pSpeed->LeftValue = pSpeed->RightValue = 0;
				} else {
					pSpeed->LeftValue = 110;
					pSpeed->RightValue = 110;
				}
			}
				break;
			case 41://����ֱ��
			{
				pSpeed->LeftValue = 120;
				pSpeed->RightValue = 120;
				//Vect_MidShoot(nRobot);
			}
				break;
			case 141://����ֱ��,�з�����
			{
				//pSpeed->LeftValue = 120;
				//pSpeed->RightValue = 120;
				Vect_MidShoot1(nRobot);
			}
				break;
			case 42://���ٻ��� �����õ�
			{
				int key;
				key = dmDEG.DEPenaltyDirection;
				switch (key) {
					case 0: {
						pSpeed->LeftValue = 0;
						pSpeed->RightValue = 0;
						
						
						//pSpeed->LeftValue = 110;
						//pSpeed->RightValue =110;
					}
						break;
					case 1: {
						pSpeed->LeftValue = 5;
						pSpeed->RightValue = 0;
						
						
						//		pSpeed->LeftValue = 135;
						//		pSpeed->RightValue =60;
					}
						break;
					case 2: {
						pSpeed->LeftValue = 20;
						pSpeed->RightValue = 20;
						
						
						//pSpeed->LeftValue = 140;
						//pSpeed->RightValue =140;
					}
						break;
					case 3: {
						pSpeed->LeftValue = 0;
						pSpeed->RightValue = 5;
						
						
						//pSpeed->LeftValue = 60;
						//pSpeed->RightValue =135;
					}
						break;
					case 4: {
						pSpeed->LeftValue = 2;
						pSpeed->RightValue = 2;
						
						//	pSpeed->LeftValue = 142;
						//	pSpeed->RightValue =142;
					}
						break;
				}
			}
				break;
			
			default: {
				char szMsg[200];
				sprintf(szMsg, "CharInterpret No Char : %d", charNo);
				::MessageBox(NULL, szMsg, "Error", MB_OK);
			}
				break;
		}
	}
}

/************************************************************************/
/*  ��ɫ���䣨������䣩�õ�����������Ӧ�õõ��Ľ�ɫ����                */
/************************************************************************/
void CDecisionMakingx::CharAllot() {
	double temp[ROBOTNUMBER] = {0};
	
	for (int i = 0; i < ROBOTNUMBER; i++)
		currentOrder[i] = 0;
	
	int nBestRobot;
	//�����ɫ����ָ��,tempjΪ��ɫ���룬temPIΪ������
	for (int tempj = 0; tempj < ROBOTNUMBER - 1; tempj++) {
		for (int temPI = 0; temPI < ROBOTNUMBER - 1; temPI++) {
			temp[temPI] = CharPerformance(currentForm[tempj], temPI);
		}
		for (i = 0; i < tempj; i++)
			temp[currentOrder[i]] = 10000;
		
		nBestRobot = GetBestRobot(temp, currentForm[tempj]);
		currentOrder[tempj] = nBestRobot;
	}
	currentOrder[ROBOTNUMBER - 1] = ROBOTNUMBER - 1;
}

/************************************************************************/
/* ��ɫָ���жϺ���                                                     */
/************************************************************************/
double CDecisionMakingx::CharPerformance(int charNo, int nRobot) {
	VecPosition posTarget;
	double angleRobot;
	int nAction2;
	BOOL bButtonSide = TRUE;
	VecPosition posBall = m_posBall;
	if (posBall.GetY() > PITCH_WIDTH / 2) {
		posBall.SetY(PITCH_WIDTH - posBall.GetY());
		bButtonSide = FALSE;
	}
	
	BOOL bHave = m_formation.GetStrategyPosition(charNo, posBall,
	                                             posTarget, angleRobot, nAction2);
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	
	double result;
	
	if (bHave) {
		if (!bButtonSide) {
			posTarget.SetY(PITCH_WIDTH - posTarget.GetY());
			angleRobot = -angleRobot;
		}
		result = GetToPositionNCycle(nRobot, posTarget);
	} else {
		switch (charNo) {
			case 0: {
				result = 0;
			}
				break;
			case 5: {
				int boundAreaNo;
				VecPosition posTarget;
				
				VecPosition posBall = m_posBall;
				double dErrorX = 10;
				double dErrorY = 0;
				if (posBall.GetX() < dErrorX)
					posBall.SetX(dErrorX);
				if (posBall.GetX() > PITCH_LENGTH - dErrorX)
					posBall.SetX(PITCH_LENGTH - dErrorX);
				if (posBall.GetY() < dErrorY)
					posBall.SetY(dErrorY);
				if (posBall.GetY() > PITCH_WIDTH - dErrorY)
					posBall.SetY(PITCH_WIDTH - dErrorY);
				boundAreaNo = GetBoundAreaNo(posBall);
				switch (boundAreaNo) {
					case 1: {
						posTarget.SetVecPosition(NUMBER2, PITCH_WIDTH + NUMBER2);
					}
						break;
					case 2: {
						posTarget.SetVecPosition(PITCH_LENGTH, PITCH_WIDTH - NUMBER2);
					}
						break;
					case 3: {
						posTarget.SetVecPosition(PITCH_LENGTH, PITCH_WIDTH / 2);
					}
						break;
					case 4: {
						posTarget.SetVecPosition(NUMBER2, NUMBER2);
					}
						break;
					case 5: {
						posTarget.SetVecPosition(PITCH_LENGTH, NUMBER2);
					}
						break;
					case 6: {
						posTarget.SetVecPosition(PITCH_LENGTH, PITCH_WIDTH / 2);
					}
						break;
					default: {
						posTarget.SetVecPosition(PITCH_LENGTH, PITCH_WIDTH / 2);
					}
						break;
				}
				
				result = GetToPositionNewPerformance(11, nRobot, posTarget, posBall);
			}
				break;
			case 6: {
				VecPosition posBall;
				result = GetKickBallPerformance(21, nRobot, posBall);
			}
				break;
			case 11: {
				VecPosition posTarget(PITCH_LENGTH + 3, PITCH_WIDTH / 2);
				VecPosition posBall;
				result = GetToPositionNewPerformance(11, nRobot, posTarget, posBall);
			}
				break;
			case 12:
				result = 100;//����Ա
				break;
			
			
			case 22: {
				VecPosition pos = GetMarkPosition();
				result = posRobot.GetDistanceTo(pos);
			}
				break;

/*		case 23:
			{
				dbPOINT	pt;
				double y;
				y = PITCH_WIDTH/2;
				pt.x = ball.x - 20;
				pt.y = -pt.x + y + PITCH_LENGTH;
				result = float(distRobot2Pt(Robot[nRobot],pt));
			}
			break;
		case 24:
			{
				dbPOINT	pt;
				double y;
				y = PITCH_WIDTH/2;
				pt.x = ball.x - 20;
				pt.y = pt.x + y - PITCH_LENGTH;
				result = float(distRobot2Pt(Robot[nRobot],pt));
			}
			break;
*/        case 39://
			{
				if (nRobot == 1)
					result = 0;
				else
					result = 10000;
			}
				break;
			
			case 40://
			{
				if (nRobot == 2)
					result = 0;
				else
					result = 10000;
			}
				break;
			case 41://
			{
				if (nRobot == 1)
					result = 0;
				else
					result = 10000;
			}
				break;
			case 42://
			{
				if (nRobot == 1)
					result = 0;
				else
					result = 10000;
			}
				break;
			default:
				result = 1000000;
		}
	}
	
	return result;
}

/************************************************************************/
/*  �õ����ŵĻ����˺���                                                */
/************************************************************************/
int CDecisionMakingx::GetBestRobot(double Performance[], int charNo) {
	double temp;
	temp = 100000;
	int BestRobotNo;
	BestRobotNo = 0;
	for (int temPI = 0; temPI < ROBOTNUMBER - 1; temPI++) {
		if (oldResult[temPI] == charNo) {
			if (perfRecord[temPI] <= Performance[temPI]
			    && Performance[temPI] > 30) {
				effecChar[charNo]++;
			} else {
				effecChar[charNo] = 0;
			}
			
			if (effecChar[charNo] < 2000) {
				Performance[temPI] *= .9;//��5%��Ϊԣ��
			} else {
				Performance[temPI] *= 2000.0;//���������ν�ɫ�жϺ�������ȡ������Ȩ
				effecChar[charNo] = 0;
			}
		}
		//����Сֵ
		if (temp > Performance[temPI]) {
			temp = Performance[temPI];
			BestRobotNo = temPI;
		}
	}
	perfRecord[BestRobotNo] = Performance[BestRobotNo];
	return BestRobotNo;
}

//�����˹�����
void CDecisionMakingx::RobotManager() {
	//���ϲ���߾����Ľ�ɫ���������ݸ�����������
	for (int i = 0; i < ROBOTNUMBER; i++) {
		currentResult[currentOrder[i]] = currentForm[i];
		m_nPriorityResult[currentOrder[i]] = m_nPriorityForm[i];
	}
	
	//���ϸ����ڵĶ��κͽ�ɫ�������洢����
	for (i = 0; i < ROBOTNUMBER; i++) {
		oldForm[i] = currentForm[i];
		oldResult[i] = currentResult[i];
		oldOrder[i] = currentOrder[i];
		oldperfRecord[i] = perfRecord[i];
	}
}

/************************************************************************/
/* �����˶���ִ��                                                       */
/************************************************************************/
void CDecisionMakingx::ActProcess() {
	for (int i = 0; i < ROBOTNUMBER; i++) {
		//rbV[i].LeftValue = rbV[i].RightValue = 0;
		CharInterpret(i, currentResult[i], &rbV[i]);
		if (currentResult[i] == 12)
			int n = 1;
		
	}
	
	int nPerfShootLine, nMinPerfShootLine = INF, nRobotShootLine = -1;
	for (i = 0; i < ROBOTNUMBER - 1; i++) {
		if (m_bRobotShoot[i]) {
			nPerfShootLine = GetShootLinePerformance(i);
			if (nPerfShootLine != -1
			    && nPerfShootLine < nMinPerfShootLine) {
				nMinPerfShootLine = nPerfShootLine;
				nRobotShootLine = i;
				SendMsg(1, "%d ShootLine Perf : %d", i, nPerfShootLine);
			}
		}
	}
	if (nRobotShootLine != -1) {
		m_bShootLine[nRobotShootLine] = TRUE;
		ShootLine(nRobotShootLine);
		SendMsg(1, "%d Shootline", nRobotShootLine);
	}
	
	
	for (i = 0; i < ROBOTNUMBER; i++)
		oldrbV[i] = rbV[i];
	
	if (bStartUnUsual)
		return;
	
	//���������ҷ��ͶԷ���������
	VecPosition posCenter(PITCH_LENGTH / 2, PITCH_WIDTH / 2);
	VecPosition pos(23, m_posBall.GetY());
	
	for (i = 0; i < ROBOTNUMBER; i++) {
		//��Ա���ҷ�����
		if (Robot[i].x < 16
		    && Robot[i].y < PITCH_WIDTH / 2 + 26 && Robot[i].y > PITCH_WIDTH / 2 - 26
		    && i != ROBOTNUMBER - 1) {
			if (Robot[i].x < Robot[ROBOTNUMBER].x + 4)
				pos.SetY(Robot[i].y);
			
			ToPositionPD(i, pos, 80);
		}
		// �Է�����
		if (Robot[i].x > PITCH_LENGTH)
			ToPositionPD(i, posCenter, 80);
		
		if (Robot[i].x > PITCH_LENGTH - 15
		    && Robot[i].y < PITCH_WIDTH / 2 + 26 && Robot[i].y > PITCH_WIDTH / 2 - 26
		    && currentResult[i] != currentForm[0]) {
			if (m_posBall.GetY() > PITCH_WIDTH / 2)
				posCenter.SetY(posCenter.GetY() - 100);
			else
				posCenter.SetY(posCenter.GetY() + 100);
			
			ToPositionPD(i, posCenter, 80);
		}
	}
	
	
}

void CDecisionMakingx::BoundPushBall(int nRobot) {
	int nBoundArea = GetBoundAreaNo(m_posBall);
	
	VecPosition posBall, posTarget, posTemp;
	VecPosition posTargetBG(195, 90);
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	
	posBall = m_posBall;
	double dErrorX = 10;
	double dErrorY = 0;
	if (posBall.GetX() < dErrorX)
		posBall.SetX(dErrorX);
	if (posBall.GetX() > PITCH_LENGTH - dErrorX)
		posBall.SetX(PITCH_LENGTH - dErrorX);
	if (posBall.GetY() < dErrorY)
		posBall.SetY(dErrorY);
	if (posBall.GetY() > PITCH_WIDTH - dErrorY)
		posBall.SetY(PITCH_WIDTH - dErrorY);
	
	const double dOffset = 0;
	
	double dist;
	double TurnV = 100;
	double distT = 6;
	
	double dx = 6;//��߽����
	double dy = 8;//�ر߽����
	
	switch (nBoundArea) {
		case 1: {
			posTemp.SetVecPosition(dOffset, dOffset);
			
			//������ǰ�Ĵ���
			if (Robot[nRobot].x < dErrorX
			    && Robot[nRobot].y <= posBall.GetY()) {
				posTarget = posBall + VecPosition(dx, dy);
				dist = (posTarget - posRobot).GetMagnitude();
				
				if (dist > 3) {
					ToPositionPD(nRobot, posTarget, MAXSPEED);
				} else {
					TurnToPointPD(nRobot, posBall, NOCLOCK);
				}
				return;
			}
			
			//�ǶȲ���ʱ�Ĵ���
			if (Robot[nRobot].x < dErrorX
			    && (Robot[nRobot].theta >= 0 && Robot[nRobot].theta < .4 * PI
			        || Robot[nRobot].theta > .6 * PI && Robot[nRobot].theta < 1.4 * PI
			        || Robot[nRobot].theta > 1.6 * PI && Robot[nRobot].theta < 2 * PI)) {
				dist = (posBall - posRobot).GetMagnitude();
				
				if (dist > distT) {
					ToPositionPD(nRobot, posBall, MAXSPEED);
				} else {
					Turn(nRobot, TurnV, ANTICLOCK);
				}
				return;
			}
			
			//���ڽ���ʱ�Ĵ���
			if (posBall.GetX() < 8
			    && posBall.GetY() < 8) {
				dist = (posBall - posRobot).GetMagnitude();
				
				if (dist > distT) {
					ToPositionPD(nRobot, posBall, MAXSPEED);
				} else {
					Turn(nRobot, TurnV, ANTICLOCK);
				}
				return;
			}
		}
			break;
		case 2: {
			posTemp.SetVecPosition(PITCH_LENGTH, dOffset);
			
			dist = (posBall - posRobot).GetMagnitude();
			
			if (m_posBall.GetX() >= PITCH_LENGTH / 2
			    && Robot[nRobot].x < m_posBall.GetX()
			    && dist <= 10
			    && ((Robot[nRobot].theta > PI * 5 / 6 && Robot[nRobot].theta <= PI)
			        || (Robot[nRobot].theta > PI * 11. / 6 && Robot[nRobot].theta <= 2 * PI))) {
				posTemp.SetVecPosition(PITCH_LENGTH + 5, PITCH_WIDTH / 2);
			}
			
			//crossing near the bottom line
			if (m_posBall.GetX() >= PITCH_LENGTH / 2
			    && Robot[nRobot].x < m_posBall.GetX()
			    && dist < 8) {
				Turn(nRobot, TurnV, ANTICLOCK);
				return;
			}
			
			//������ǰ�Ĵ���
			if (Robot[nRobot].y < dErrorX
			    && Robot[nRobot].x >= posBall.GetX()) {
				posTarget = posBall + VecPosition(-dx, dy);
				
				dist = (posTarget - posRobot).GetMagnitude();
				
				if (dist > 3) {
					ToPositionNew(nRobot, posTargetBG, posBall, MAXSPEED, 0);
				} else {
					TurnToPointPD(nRobot, posBall, NOCLOCK);
				}
				return;
			}
			
			//�ǶȲ���ʱ�Ĵ���
			if (Robot[nRobot].y < dErrorX &&
			    (Robot[nRobot].theta > .1 * PI && Robot[nRobot].theta < .9 * PI
			     || Robot[nRobot].theta > 1.1 * PI && Robot[nRobot].theta < 1.9 * PI)) {
				dist = (posBall - posRobot).GetMagnitude();
				
				if (dist > distT) {
					ToPositionPD(nRobot, posBall, MAXSPEED);
				} else {
					Turn(nRobot, TurnV, ANTICLOCK);
				}
				return;
			}
			
			//���ڽ���ʱ�Ĵ���
			if (posBall.GetX() > PITCH_LENGTH - 8
			    && posBall.GetY() < 8) {
				dist = (posBall - posRobot).GetMagnitude();
				
				if (dist > distT) {
					ToPositionPD(nRobot, posBall, MAXSPEED);
				} else {
					Turn(nRobot, TurnV, ANTICLOCK);
				}
				return;
			}
		}
			break;
		case 3: {
			posTemp.SetVecPosition(PITCH_LENGTH, PITCH_WIDTH / 2 + 15);
			
			//crossing near the bottom line
			dist = (posRobot - posBall).GetMagnitude();
			
			if (Robot[nRobot].y < posBall.GetY() &&
			    dist < 8) {
				Turn(nRobot, TurnV, ANTICLOCK);
				return;
			}
			
			//������ǰ�Ĵ���
			if (Robot[nRobot].x > PITCH_LENGTH - dErrorX
			    && Robot[nRobot].y >= posBall.GetY()) {
				posTarget = posBall + VecPosition(-dx, -dy);
				
				dist = (posTarget - posRobot).GetMagnitude();
				
				if (dist > 2) {
					ToPositionNew(nRobot, posTargetBG, posBall, MAXSPEED, 0);
				} else {
					TurnToPointPD(nRobot, posBall, NOCLOCK);
				}
				return;
			}
			
			//�ǶȲ���ʱ�Ĵ���
			if (Robot[nRobot].x > PITCH_LENGTH - dErrorX
			    && (Robot[nRobot].theta >= 0 && Robot[nRobot].theta < .4 * PI
			        || Robot[nRobot].theta > .6 * PI && Robot[nRobot].theta < 1.4 * PI
			        || Robot[nRobot].theta > 1.6 * PI && Robot[nRobot].theta < 2 * PI)) {
				dist = (posBall - posRobot).GetMagnitude();
				
				if (dist > distT) {
					ToPositionPD(nRobot, posBall, MAXSPEED);
				} else {
					Turn(nRobot, TurnV, ANTICLOCK);
				}
				return;
			}
		}
			break;
		case 4: {
			posTemp.SetVecPosition(dOffset, PITCH_WIDTH + dOffset);
			
			//������ǰ�Ĵ���
			if (Robot[nRobot].x < dErrorX
			    && Robot[nRobot].y >= posBall.GetY()) {
				posTarget = posBall + VecPosition(dx, -dy);
				dist = (posTarget - posRobot).GetMagnitude();
				
				if (dist > 2) {
					ToPositionPD(nRobot, posTarget, MAXSPEED);
				} else {
					TurnToPointPD(nRobot, posBall, NOCLOCK);
				}
				return;
			}
			
			//�ǶȲ���ʱ�Ĵ���
			if (Robot[nRobot].x < dErrorX
			    && (Robot[nRobot].theta >= 0 && Robot[nRobot].theta < .4 * PI
			        || Robot[nRobot].theta > .6 * PI && Robot[nRobot].theta < 1.4 * PI
			        || Robot[nRobot].theta > 1.6 * PI && Robot[nRobot].theta < 2 * PI)) {
				dist = (posBall - posRobot).GetMagnitude();
				
				if (dist > distT) {
					ToPositionPD(nRobot, posBall, MAXSPEED);
				} else {
					Turn(nRobot, TurnV, CLOCKWISE);
				}
				return;
			}
			
			//���ڽ���ʱ�Ĵ���
			if (posBall.GetX() < 8
			    && posBall.GetY() > PITCH_WIDTH - 8) {
				dist = (posBall - posRobot).GetMagnitude();
				
				if (dist > distT) {
					ToPositionPD(nRobot, posBall, MAXSPEED);
				} else {
					Turn(nRobot, TurnV, CLOCKWISE);
				}
				return;
			}
		}
			break;
		case 5: {
			posTemp.SetVecPosition(PITCH_LENGTH, PITCH_WIDTH - dOffset);
			
			dist = (posBall - posRobot).GetMagnitude();
			
			if (m_posBall.GetX() >= PITCH_LENGTH / 2
			    && Robot[nRobot].x < m_posBall.GetX()
			    && dist <= 10
			    && ((Robot[nRobot].theta > PI * 5 / 6 && Robot[nRobot].theta <= PI)
			        || (Robot[nRobot].theta > PI * 11. / 6 && Robot[nRobot].theta <= 2 * PI))) {
				posTemp.SetVecPosition(PITCH_LENGTH + 5, PITCH_WIDTH / 2);
			}
			
			//crossing near the bottom line
			if (m_posBall.GetX() >= PITCH_LENGTH / 2
			    && Robot[nRobot].x < m_posBall.GetX()
			    && dist < 8) {
				Turn(nRobot, TurnV, CLOCKWISE);
				return;
			}
			
			//������ǰ�Ĵ���
			if (Robot[nRobot].y > PITCH_WIDTH - dErrorX
			    && Robot[nRobot].x >= posBall.GetX()) {
				posTarget = posBall - VecPosition(dx, dy);
				
				dist = (posTarget - posRobot).GetMagnitude();
				
				if (dist > 3) {
					ToPositionNew(nRobot, posTargetBG, posBall, MAXSPEED, 0);
				} else {
					TurnToPointPD(nRobot, posBall, NOCLOCK);
				}
				return;
			}
			
			//�ǶȲ���ʱ�Ĵ���
			if (Robot[nRobot].y > PITCH_WIDTH - dErrorX &&
			    (Robot[nRobot].theta > .1 * PI && Robot[nRobot].theta < .9 * PI
			     || Robot[nRobot].theta > 1.1 * PI && Robot[nRobot].theta < 1.9 * PI)) {
				dist = (posBall - posRobot).GetMagnitude();
				
				if (dist > distT) {
					ToPositionPD(nRobot, posBall, MAXSPEED);
				} else {
					Turn(nRobot, TurnV, CLOCKWISE);
				}
				return;
			}
			
			//���ڽ���ʱ�Ĵ���
			if (posBall.GetX() > PITCH_LENGTH - 8
			    && posBall.GetY() > PITCH_WIDTH - 8) {
				dist = (posBall - posRobot).GetMagnitude();
				
				if (dist > distT) {
					ToPositionPD(nRobot, posBall, MAXSPEED);
				} else {
					Turn(nRobot, TurnV, CLOCKWISE);
				}
				return;
			}
		}
			break;
		case 6: {
			posTemp.SetVecPosition(PITCH_LENGTH, PITCH_WIDTH / 2 - 15);
			
			//crossing near the bottom line
			dist = (posRobot - posBall).GetMagnitude();
			
			if (Robot[nRobot].y > posBall.GetY() &&
			    dist < 8) {
				Turn(nRobot, TurnV, CLOCKWISE);
				return;
			}
			
			//������ǰ�Ĵ���
			if (Robot[nRobot].x > PITCH_LENGTH - dErrorX
			    && Robot[nRobot].y <= posBall.GetY()) {
				posTarget = posBall + VecPosition(-dx, dy);
				
				dist = (posTarget - posRobot).GetMagnitude();
				
				if (dist > 2) {
					ToPositionNew(nRobot, posTargetBG, posBall, MAXSPEED, 0);
				} else {
					TurnToPointPD(nRobot, posBall, NOCLOCK);
				}
				return;
			}
			
			//�ǶȲ���ʱ�Ĵ���
			if (Robot[nRobot].x > PITCH_LENGTH - dErrorX
			    && (Robot[nRobot].theta >= 0 && Robot[nRobot].theta < .4 * PI
			        || Robot[nRobot].theta > .6 * PI && Robot[nRobot].theta < 1.4 * PI
			        || Robot[nRobot].theta > 1.6 * PI && Robot[nRobot].theta < 2 * PI)) {
				dist = (posBall - posRobot).GetMagnitude();
				
				if (dist > distT) {
					ToPositionPD(nRobot, posBall, 70);
				} else {
					Turn(nRobot, TurnV, CLOCKWISE);
				}
				return;
			}
		}
			break;
		
		default:
			break;
	}
	
	ToPositionNew(nRobot, posTemp, m_posBall, MAXSPEED, 1);
	return;
}


int CDecisionMakingx::GetBoundAreaNo(VecPosition posBall) {
	BOOL bButtonSide;
	if (posBall.IsTopOf(PITCH_WIDTH / 2)) {
		posBall.SetY(PITCH_WIDTH - posBall.GetY());
		bButtonSide = FALSE;
	} else {
		bButtonSide = TRUE;
	}
	
	int nBoundArea = 0;
	
	if (posBall.GetX() < 15)
		nBoundArea = 1;
	else if (posBall.GetY() < 15)
		nBoundArea = 2;
	else
		nBoundArea = 3;
	
	if (!bButtonSide)
		nBoundArea += 3;
	
	return nBoundArea;
}

double CDecisionMakingx::Getpt2ptAngle(dbPOINT pt1, dbPOINT pt2) {
	double angle;
	double dx, dy;
	dx = pt2.x - pt1.x;
	dy = pt2.y - pt1.y;
	angle = atan2(dy, dx);
	angle = VecPosition::NormalizeAngle2PI(angle);
	return angle;
}

void CDecisionMakingx::PushBallofBound(int nRobot) {
	int nBoundAreaNo = GetBoundAreaNo(m_posBall);
	
	VecPosition posTarget;
	switch (nBoundAreaNo) {
		case 1: {
			posTarget.SetVecPosition(PITCH_LENGTH / 4, PITCH_WIDTH * 5 / 6);
		}
			break;
		case 2:
		case 5: {
			posTarget.SetVecPosition(PITCH_LENGTH, PITCH_WIDTH / 2);
		}
			break;
		case 3:
		case 6: {
			posTarget.SetVecPosition(PITCH_LENGTH * 5 / 6, PITCH_WIDTH / 2);
		}
			break;
		case 4: {
			posTarget.SetVecPosition(PITCH_LENGTH / 4, PITCH_WIDTH / 6);
		}
			break;
		default:
			break;
	}
	
	ToPositionNew(nRobot, posTarget, m_posBall, 70, 2);
	
	return;
}

/************************************************************************/
/* ���ź����ƽ�ɫ�жϺ���                                               */
/* �ɵ����� : k, k1,													*/
/************************************************************************/
//DEL double CDecisionMakingx::GetCharPerformance(int no, RobotInford* myRobot, VecPosition posBall, VecPosition posTarget)
//DEL {
//DEL 	VecPosition posRobot(myRobot->x, myRobot->y);
//DEL 	double angleBall2Target = (posTarget - posBall).GetDirection();
//DEL 	double angleRobot2Ball = (posBall - posRobot).GetDirection();
//DEL 	double dist = (posRobot - posBall).GetMagnitude();
//DEL
//DEL 	double angle = angleBall2Target - angleRobot2Ball;
//DEL 	double angle2 = angleRobot2Ball - myRobot->theta;
//DEL
//DEL 	angle = VecPosition::NormalizeAngle(angle);
//DEL 	angle2 = VecPosition::NormalizeAngle(angle2);
//DEL 	if(angle2 > PI/2)
//DEL 		angle2 = PI - angle2;
//DEL
//DEL 	double k = 20;
//DEL 	if(no == 5)
//DEL 		k = 10;
//DEL
//DEL 	double k1 = 2;
//DEL
//DEL 	double result = k*angle + k1*angle2 + dist;
//DEL
//DEL 	return result;
//DEL }


/************************************************************************/
/* Ԥ������nCycle�����ں��λ��                                         */
/************************************************************************/
VecPosition CDecisionMakingx::PredictBall(int nCycle, VecPosition *posVel) {
	if (nCycle < 0)
		nCycle = 0;
	VecPosition posBall = m_posBall + VecPosition(ballCharacter.velocity * cos(ballCharacter.angle),
	                                              ballCharacter.velocity * sin(ballCharacter.angle)) * nCycle;
	if (posVel != NULL)
		posVel->SetVecPosition(ballCharacter.velocity * cos(ballCharacter.angle),
		                       ballCharacter.velocity * sin(ballCharacter.angle));
	if (posBall.GetX() < 0)
		posBall.SetX(-posBall.GetX());
	if (posBall.GetX() > PITCH_LENGTH)
		posBall.SetX(2 * PITCH_LENGTH - posBall.GetX());
	if (posBall.GetY() < 0)
		posBall.SetY(-posBall.GetY());
	if (posBall.GetY() > PITCH_WIDTH)
		posBall.SetY(2 * PITCH_WIDTH - posBall.GetY());
	
	return posBall;
}

int CDecisionMakingx::KalmanFilter() {
	int n = 1;
	double K[3][1];
	double TransferK[1][3];
	double TransferY[1][2];
	double TransferX[3][2];
	double TransferP[3][3];
	double Param[1][1];
	double Parameter;
	double PeriodTime = 0.01666;//33.33ms
	double ForeA[3][3] = {{1.00, n * PeriodTime, n * n * PeriodTime * PeriodTime / 2},
	                      {0.00, 1.00,           n * PeriodTime},
	                      {0.00, 0.00,           1.00}};
	double ForeX[3][2];
	//	double ForeX[3][3];
	//	double ForeA[3][3]={{1.00,n*PeriodTime,n*n*PeriodTime*PeriodTime/2},{0.00,1.00,n*PeriodTime},{0.00,0.00,1.00}};
	
	double X[3][2] = {{oldBallPt[0].x, oldBallPt[0].y},
	                  {0.00,           0.00},
	                  {0.00,           0.00}};
	
	double Rk = 0.331859784;
	
	double P[3][3] = {{200.00, 0.00,   0.00},
	                  {0.00,   200.00, 0.00},
	                  {0.00,   0.00,   200.00}};
	
	double A[3][3] = {{1.00, PeriodTime, PeriodTime * PeriodTime / 2},
	                  {0.00, 1.00,       PeriodTime},
	                  {0.00, 0.00,       1.00}};
	
	double C[1][3] = {1.00, 0.00, 0.00};
	
	double AT[3][3] = {{1.00,                        0.00,       0.00},
	                   {PeriodTime,                  1.00,       0.00},
	                   {PeriodTime * PeriodTime / 2, PeriodTime, 1.00}};
	
	double CT[3][1] = {{1.00},
	                   {0.00},
	                   {0.00}};
	
	double E[3][3] = {{1.00, 0.00, 0.00},
	                  {0.00, 1.00, 0.00},
	                  {0.00, 0.00, 1.00}};
	
	
	for (int Mi = 1; Mi < 7; Mi++) {
		if (oldBallPt[Mi].x == 0.00 && oldBallPt[Mi].y == 130.00)
			oldBallPt[Mi] = oldBallPt[Mi - 1];
		double Y[1][2] = {oldBallPt[Mi].x, oldBallPt[Mi].y};
		//calculate the "Pk|k-1"
		Matrix::MtxMultiply(*A, *P, 3, 3, 3, *P);
		Matrix::MtxMultiply(*P, *AT, 3, 3, 3, *P);
		//calculate the "K"
		Matrix::MtxMultiply(*C, *P, 1, 3, 3, *TransferK);
		Matrix::MtxMultiply(*TransferK, *CT, 1, 3, 1, *Param);
		Parameter = 1 / (Param[0][0] + Rk);
		Matrix::MtxMultiply(*P, *CT, 3, 3, 1, *K);
		Matrix::MtxParaMultiply(*K, Parameter, 3, 1, *K);
		//calculate the "X"
		Matrix::MtxMultiply(*A, *X, 3, 3, 2, *X);
		Matrix::MtxMultiply(*C, *X, 1, 3, 2, *TransferY);
		Matrix::MtxSubtract(*Y, *TransferY, 1, 2, *Y);
		Matrix::MtxMultiply(*K, *Y, 3, 1, 2, *TransferX);
		Matrix::MtxPlus(*X, *TransferX, 3, 2, *X);
		
		FilterBallPt[Mi].x = X[0][0];
		FilterBallPt[Mi].y = X[0][1];
		FilterVel[Mi].x = X[1][0];
		FilterVel[Mi].y = X[1][1];//save the value after filter
		FilterAccel[Mi].x = X[2][0];
		FilterAccel[Mi].y = X[2][1];
		
		//calculate the "P"
		Matrix::MtxMultiply(*K, *C, 3, 1, 3, *TransferP);
		Matrix::MtxSubtract(*E, *TransferP, 3, 3, *TransferP);
		Matrix::MtxMultiply(*TransferP, *P, 3, 3, 3, *P);
	}
	Matrix::MtxMultiply(*ForeA, *X, 3, 3, 2, *ForeX);

//	ForeBallPt.x=ForeX[0][0];	ForeBallPt.y=ForeX[0][1];
//	ForeVel.x=ForeX[1][0];      ForeVel.y=ForeX[1][1];
	
	return 0;
}

/************************************************************************/
/* ��¼��������															*/
/************************************************************************/
void CDecisionMakingx::LogData() {
/*** following is to send the game info to the logplayer or write the log file ************************************/
	dispinfo_t disPInfo;
	
	disPInfo.mode = SHOW_MODE;
	disPInfo.body.show.time = m_nTimer;
	
	// because of the coordinate system is different, we should transfer it first.
	// in here, origin is the left-button point
	// in to-info, origin is the left-top point
	for (int i = 0; i <= 2 * ROBOTNUMBER; i++) {
		disPInfo.body.show.pos[i].x = Robot[i].x;
		disPInfo.body.show.pos[i].y = Robot[i].y;
		disPInfo.body.show.pos[i].angle = Robot[i].theta;
	}
	
	// send the display info
	log.SendMessageOfGame(&disPInfo);
	
	// or write the display info do the file directly
	//	fwrite(&disPInfo, sizeof(dispinfo_t), 1, m_file);
}

/************************************************************************/
/* ��С����Ԥ����Ĺ켣                                                 */
/************************************************************************/
int CDecisionMakingx::Forcastball(dbPOINT *pPoint, FORCASTBALL *pBall) {
	double aver_x, aver_y;
	double aver_xy, aver_xx;
	dbPOINT point1, point2;
	Formulation formu1, formu2;
	aver_x = 0;
	aver_y = 0;
	aver_xy = 0;
	aver_xx = 0;
	
	for (int i = 0; i < BALLNUMPARA; i++) {
		aver_x += oldBallPt[i].x;
		aver_y += oldBallPt[i].y;
		aver_xy += oldBallPt[i].x * oldBallPt[i].y;
		aver_xx += oldBallPt[i].x * oldBallPt[i].x;
	}
	
	aver_x = aver_x / BALLNUMPARA;
	aver_y = aver_y / BALLNUMPARA;
	aver_xy = aver_xy / BALLNUMPARA;
	aver_xx = aver_xx / BALLNUMPARA;
	
	double temp;
	temp = aver_xx - aver_x * aver_x;
	
	if (temp == 0) {
		pBall->velocity = 0.0;
		pBall->Formu.a = 0.0;
		pBall->Formu.b = 0.0;
		pBall->Formu.c = 0.0;
		return 1;
	}
	pBall->Formu.a = (aver_xy - aver_x * aver_y) / temp;
	pBall->Formu.c = aver_y - pBall->Formu.a * aver_x;
	pBall->Formu.b = -1;
	//��һ�㴹ֱ��һ��ֱ�ߵķ��̺ͽ���
	cn_PointPerpendLine(*(pPoint + BALLNUMPARA - 1), &(pBall->Formu), &formu1, &point1);
	//��һ�㴹ֱ��һ��ֱ�ߵķ��̺ͽ���
	cn_PointPerpendLine(*(pPoint), &(pBall->Formu), &formu2, &point2);
	
	pBall->angle = cn_LineAngle(point2, point1);
	pBall->velocity = cn_2PointsDist(oldBallPt[4], oldBallPt[6]) / 2;
	
	double dx = oldBallPt[6].x - oldBallPt[0].x;
	double dy = oldBallPt[6].y - oldBallPt[0].y;
	
	pBall->proBall.x = oldBallPt[6].x + dx;
	pBall->proBall.y = oldBallPt[6].y + dy;
	
	return 1;
}

/************************************************************************/
/* ����(���dMadSpeed)                                                  */
/* bMax = TRUE -> ��֤���ٶ�,������ٶ�									*/
/************************************************************************/
void CDecisionMakingx::LimitSpeed(dbLRWheelVelocity *pSpeed, double dMaxSpeed, BOOL bMax) {
	double speed_e = pSpeed->LeftValue - pSpeed->RightValue;
	
	if (speed_e > 2 * dMaxSpeed)
		speed_e = 2 * dMaxSpeed;
	
	if (fabs(pSpeed->LeftValue) > fabs(pSpeed->RightValue)) {
		if (pSpeed->LeftValue > dMaxSpeed) {
			pSpeed->LeftValue = dMaxSpeed;
			pSpeed->RightValue = pSpeed->LeftValue - speed_e;
		} else if (pSpeed->LeftValue < -dMaxSpeed) {
			pSpeed->LeftValue = -dMaxSpeed;
			pSpeed->RightValue = pSpeed->LeftValue - speed_e;
		}
		
	} else {
		if (pSpeed->RightValue > dMaxSpeed) {
			pSpeed->RightValue = dMaxSpeed;
			pSpeed->LeftValue = pSpeed->RightValue + speed_e;
		} else if (pSpeed->RightValue < -dMaxSpeed) {
			pSpeed->RightValue = -dMaxSpeed;
			pSpeed->LeftValue = pSpeed->RightValue + speed_e;
		}
	}
	
	if (bMax) {
		if (fabs(pSpeed->LeftValue) > fabs(pSpeed->RightValue)) {
			if (pSpeed->LeftValue > 0)
				pSpeed->LeftValue = dMaxSpeed;
			else
				pSpeed->LeftValue = -dMaxSpeed;
			
			pSpeed->RightValue = pSpeed->LeftValue - speed_e;
		} else {
			if (pSpeed->RightValue > 0)
				pSpeed->RightValue = dMaxSpeed;
			else
				pSpeed->RightValue = -dMaxSpeed;
			
			pSpeed->LeftValue = pSpeed->RightValue + speed_e;
		}
	}
}

/************************************************************************/
/* ����ToPositionN����	  									            */
/************************************************************************/
int CDecisionMakingx::GetToPositionNCycle(int nRobot, VecPosition posTarget) {
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	
	double dist = (posRobot - posTarget).GetMagnitude();
	double angleRobot2Ball = (posTarget - posRobot).GetDirection();
	double angle = angleRobot2Ball - Robot[nRobot].theta;
	angle = fabs(VecPosition::NormalizeAngle(angle));
	if (angle > PI / 2)
		angle = PI - angle;
	
	// �˴�����Matlab���
	/*	nRobotCycle2 = (int) (dist * 0.3064
	+  angle * 10.5972
	+ Robot[nRobot].speed * (-1.0898));
	*/

#if 1
	// BP�������
	double input[3], output;
	input[0] = dist / 200.;
	input[1] = angle / PI * 2;
	input[2] = Robot[nRobot].speedv / 5;
	nettpt.compute(input, &output);
	int nRobotCycle = static_cast<int>(output * 60);
#else
																															// SVM ���
	m_svmAttrTpt[0].index = 1;
	m_svmAttrTpt[0].value = dist/200.;
	m_svmAttrTpt[1].index = 2;
	m_svmAttrTpt[1].value = angle/PI*2;
	m_svmAttrTpt[2].index = 3;
	m_svmAttrTpt[2].value = Robot[nRobot].speed/5;
	m_svmAttrTpt[3].index = -1;
	double v = svm_predict(m_svmModelTpt, m_svmAttrTpt);
	int nRobotCycle = static_cast<int>(v * 60);
#endif
	
	return nRobotCycle;
}
/************************************************************************/
/* ����ToPositionNew����	  								            */
/************************************************************************/
int CDecisionMakingx::GetToPositionNewCycle(int nRole, int nRobot,
                                            VecPosition posBall, VecPosition posTarget) {
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	
	double distBall2Robot = (posBall - posRobot).GetMagnitude();
	double angleBall2Robot = (posBall - posRobot).GetDirection();
	double angleTarget2Ball = (posTarget - posBall).GetDirection();
	
	double angle1 = angleBall2Robot - Robot[nRobot].theta;
	double angle2 = angleTarget2Ball - angleBall2Robot;
	
	angle1 = fabs(VecPosition::NormalizeAngle(angle1));
	if (angle1 > PI / 2)
		angle1 = PI - angle1;
	angle2 = fabs(VecPosition::NormalizeAngle(angle2));
	if (angle2 > PI / 2)
		angle2 = PI - angle2;
	
	// Matlab���
	/*	nRobotCycle = (int) (distBall2Robot * 0.3056
	+ angle1 * 4.4571 + angle2 * 7.7470
	+ Robot[nRobot].speed * (-1.8782));
	*/
	// BP�������
	double input[4], output;
	input[0] = distBall2Robot / 200;
	input[1] = angle1 / PI * 2;
	input[2] = angle2 / PI * 2;
	input[3] = Robot[nRobot].speedv / 5;
	nettpn.compute(input, &output);
	int nRobotCycle = static_cast<int>(output * 60);
	
	return nRobotCycle;
}

/************************************************************************/
/* �����������ToPositionNew)�ص�(PosBall)������(����ֵ)              */
/************************************************************************/
int CDecisionMakingx::GetToPositionNewPerformance(int nRole, int nRobot,
                                                  VecPosition posTarget, VecPosition &posBall) {
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	
	posBall = m_posBall;
	
	double angle1 = VecPosition::NormalizeAngle(
			(posTarget - posBall).GetDirection() - (posBall - posRobot).GetDirection());
	double angle2 = VecPosition::NormalizeAngle((posBall - posRobot).GetDirection() - Robot[nRobot].theta);
	angle1 = fabs(angle1);
	angle2 = fabs(angle2);
	if (angle2 > PI / 2)
		angle2 = PI - angle2;
	
	return static_cast<int>( posRobot.GetDistanceTo(posBall) + 4 * angle1 + 0 * angle2);
	
	
	// �����������ʱ��
	int nCycle = -1;
	int nRobotCycle;
	
	// ��������˵���켣���߽�������Ҫ������
	Line lineBall(ballCharacter.Formu.b, ballCharacter.Formu.a, ballCharacter.Formu.c);
	VecPosition posPerpBall = lineBall.GetPointOnLineClosestTo(posRobot);
	double dDistMax = 2 * (posPerpBall - m_posBall).GetMagnitude();
	double dDist;
	do {
		nCycle++;
		
		posBall = PredictBall(nCycle);
		
		nRobotCycle = GetToPositionNewCycle(11, nRobot, posBall, posTarget);
		
		dDist = (posBall - m_posBall).GetMagnitude();
	} while (nCycle < nRobotCycle && dDist < dDistMax);

//	SendMsg(2, "%d %d %2.2f %2.2f, %2.2f %2.2f",
//		nCycle, nRobotCycle, dDist, dDistMax, posBall.GetX(), posBall.GetY());
	
	
	return nRobotCycle;
}

/************************************************************************/
/* �����������ToPositionN)�ص�(PosBall)������(����ֵ)                */
/************************************************************************/
int CDecisionMakingx::GetKickBallPerformance(int nRole, int nRobot, VecPosition &posBall) {
	// �����������ʱ��
	VecPosition posRobot(Robot[nRobot].x, Robot[nRobot].y);
	posBall = m_posBall;
	
	return static_cast<int>(posRobot.GetDistanceTo(posBall));
	
	int nCycle = -1;
	int nRobotCycle;
	
	// ��������˵���켣���߽�������Ҫ������
	Line lineBall(ballCharacter.Formu.b, ballCharacter.Formu.a, ballCharacter.Formu.c);
	VecPosition posPerpBall = lineBall.GetPointOnLineClosestTo(posRobot);
	double dDistMax = 2 * (posPerpBall - m_posBall).GetMagnitude();
	double dDist;
	do {
		nCycle++;
		
		posBall = PredictBall(nCycle);
		
		nRobotCycle = GetToPositionNCycle(nRobot, posBall);
		
		dDist = (posBall - m_posBall).GetMagnitude();
	} while (nCycle < nRobotCycle && dDist < dDistMax);

//	SendMsg(2, "%d %d", nCycle, nRobotCycle);
	
	return nRobotCycle;
}

/************************************************************************/
/* ��ʱ��                                                               */
/************************************************************************/
void CDecisionMakingx::Update(CCounterTimer *Timer) {
	if (Timer == &m_timerStart) {
		bStartUnUsual = FALSE;
		m_nTimerLastStart = -1;
	} else if (Timer == &m_timerShootLine) {
		for (int i = 0; i < ROBOTNUMBER; i++)
			m_bShootLine[i] = FALSE;
		
		m_timerShootLine.Stop();
	} else if (Timer == &m_timerBallKick) {
		m_bBallKick = FALSE;
		m_timerBallKick.Stop();
	}
}

void CDecisionMakingx::Capture() {
#ifdef XIMAGE
																															HWND hwnd = ::FindWindow(NULL, "RobotSoccer");
	if (hwnd == NULL)
		return;

		// get window size
	RECT r;
	::GetWindowRect(hwnd,&r);

	int xScreen,yScreen;	//check if the window is out of the screen or maximixed <Qiang>
	int xshift = 0, yshift = 0;
	xScreen = GetSystemMetrics(SM_CXSCREEN);
	yScreen = GetSystemMetrics(SM_CYSCREEN);
	if(r.right > xScreen)
		   r.right = xScreen;
	if(r.bottom > yScreen)
		   r.bottom = yScreen;
	if(r.left < 0)
	{
		xshift = -r.left;
		r.left = 0;
	}
	if(r.top < 0)
	{
		yshift = -r.top;
		r.top = 0;
	}
	
	SIZE sz;
	sz.cx = r.right - r.left;
	sz.cy = r.bottom - r.top;

	if(sz.cx <= 0 || sz.cy <= 0) return;
	
	// bring the window at the top most level
	::SetWindowPos(hwnd,HWND_TOPMOST,0,0,0,0,SWP_NOMOVE|SWP_NOSIZE);
	
	// prepare the DCs
	HDC dstDC = ::GetDC(NULL);
	HDC srcDC = ::GetWindowDC(hwnd); //full window (::GetDC(hwnd); = clientarea)
	HDC memDC = ::CreateCompatibleDC(dstDC);
	
	// copy the screen to the bitmap
	HBITMAP bm =::CreateCompatibleBitmap(dstDC, sz.cx, sz.cy);
	HBITMAP oldbm = (HBITMAP)::SelectObject(memDC,bm);
	::BitBlt(memDC, 0, 0, sz.cx, sz.cy, srcDC, xshift, yshift, SRCCOPY);
	
	// restore the position
	::SetWindowPos(hwnd,HWND_NOTOPMOST,0,0,0,0,SWP_NOMOVE|SWP_NOSIZE);
//	::SetWindowPos(m_hWnd,HWND_TOP,0,0,0,0,SWP_NOMOVE|SWP_NOSIZE);
	
	CxImage *newima = new CxImage();
	newima->CreateFromHBITMAP(bm);
	char strFileName[10];
	sprintf(strFileName, "c:\\%d.jpg", m_nTimer/10);
	newima->Save(strFileName, CXIMAGE_FORMAT_JPG);
	// free objects
	DeleteObject(SelectObject(memDC,oldbm));
	DeleteObject(memDC);
#endif
}

void CDecisionMakingx::SendMsg(int nLevel, char *msg, ...) {
//	return;
	
	char szMsg[500];
	va_list ap;
	va_start(ap, msg);
	vsprintf(szMsg, msg, ap);
	va_end(ap);
	
	log.SendMsg(m_nTimer, nLevel, szMsg);
}

/************************************************************************/
/* ����																	*/
/************************************************************************/
void CDecisionMakingx::Mark(int nRobot) {
	VecPosition posTarget = GetMarkPosition();
	Wait(nRobot, posTarget, 0, 100);
}

VecPosition CDecisionMakingx::GetMarkPosition() {
	VecPosition posOwnGoal(0, PITCH_WIDTH / 2);
	
	// find nearest enemy
	double dDist = INF;
	VecPosition posEnemy, posMinEnemy;
	for (int i = 0; i < ROBOTNUMBER; i++) {
		posEnemy.SetVecPosition(Robot[i + ROBOTNUMBER].x, Robot[i + ROBOTNUMBER].y);
		if (posOwnGoal.GetDistanceTo(posEnemy) < dDist) {
			dDist = posOwnGoal.GetDistanceTo(posEnemy);
			posMinEnemy = posEnemy;
		}
	}
	return posMinEnemy;
}


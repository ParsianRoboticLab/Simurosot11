#ifndef _GLOBE_H_
#define _GLOBE_H_

#include "math.h"

#define PI 3.14159265358979323846
#define sqrt2 1.414213562373

#define CLOCKWISE 1
#define ANTICLOCK -1
#define NOCLOCK  0

typedef struct {
	double x;
	double y;
} dbPOINT, BallInformation, ballInformation;

//�������帡��λ����Ϣ
typedef struct {
	double x;    //�������ĵ�����x
	double y;  //�������ĵ�����y,������������ʾ����Ϊ��׼��
	double theta;//���ȷ����
	double speedv;    // ���������ٶȣ������ڣ�
	double speedw;    // �����˽��ٶ�
} dbROBOTPOSTURE, RobotInford;

//С������ֵ������
typedef struct {
	double LeftValue;
	double RightValue;
} dbLRWheelVelocity;

typedef struct {
	double a;
	double b;
	double c;
} Formulation, LINEFORMULATION;

typedef struct {
	Formulation Formu;
	double angle;
	double velocity;
	dbPOINT proBall;
} FORCASTBALL;

struct DEGame {
	int DEGameGround;
	int DEStartState;
	int DEStartMode;
	int DEPenaltyDirection;
};

#endif    // _GLOBE_H_

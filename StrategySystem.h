// StrategySystem.h
#include "general.h"

#ifndef _INSIDE_VISUAL_CPP_STRATEGYSYSTEM
#define _INSIDE_VISUAL_CPP_STRATEGYSYSTEM
//////////////////////////////////////////////////NEW/////////////////////////////////////////////////////

class CStrategySystem : public CObject {
	DECLARE_DYNAMIC(CStrategySystem)
public:
	~CStrategySystem();
	
	CStrategySystem(int id);
	
	void ReceiveData(Ball bal, OurRobot ho1, OurRobot ho2, OurRobot ho3, OurRobot ho4,
	                 OurRobot ho5, OurRobot ho6, OurRobot ho7, OurRobot ho8, OurRobot ho9,
	                 OurRobot ho10, OurRobot hgo, Opponent opp);

#ifdef _DEBUG
#endif // _DEBUG
private:
	
	void GoaliePosition(int which, CPoint point);
	
	void Stop(int which);
	
	void Goalie(int which);
	
	void Position(int which, CPoint point);
	
	void Velocity(int which, int vL, int vR);
	
	void Angle(int which, int desired_angle);
	
	CRect boundRect;
	int m_nStrategy;
	int m_OurTeam;
	int m_nGameArea;
	double ShootLen;                  //������ľ���
	int nShoot;                    //�����״̬����
	int nSweep;
	int nKick2;
	CPoint ShootVar;                  //���λ��
	CPoint KickVar;
	double KickLen;
/*	CCommand C_Home3;
	CCommand C_Home2;
	CCommand C_Home1;
	CCommand C_Home4;
	CCommand C_Home5;
	CCommand C_Home6;
	CCommand C_Home7;
	CCommand C_Home8;
	CCommand C_Home9;
	CCommand C_Home10;
	CCommand C_Home11;*/
	Ball ball;
	OurRobot home1, home2, home3, home4, home5, home6, home7, home8, home9, home10, hgoalie;
	Opponent opponent;
	
	//unsigned char command[11];
	void Think();
	
	void NormalGame5();

public:
	void Action();
	
	int CountDistance(CPoint point1, CPoint point2);
	
	CPoint C_NearestOppFromBall();
	
	void C_KickTo(int which, CPoint pos);
	
	int C_CheckBallPos();
	
	int command[35];
	
};

#endif // _INSIDE_VISUAL_CPP_STRATEGYSYSTEM

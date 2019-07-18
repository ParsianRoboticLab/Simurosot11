#include "StdAfx.h"
#include "StrategySystem.h"
//#include "Communications.h"

IMPLEMENT_DYNAMIC(CStrategySystem, CObject
)

extern int nKick;


#define  BALL_WIDTH        78//30
#define  BALL_LENGTH    156//60
#define  BALL_DIS        26//10
#define  CORNER         115//45

CStrategySystem::CStrategySystem(int id) {
	m_OurTeam = id;
	
	boundRect.SetRect(65, 95, 965, 723);//�����Ľ�
	
	if (id)
		m_nGameArea = GAME_LEFT;
	else
		m_nGameArea = GAME_RIGHT;
	for (int i = 0; i < 35; i++)
		command[i] = 0;
/*	C_Home1.Data.Lv=0;
	C_Home1.Data.Rv=0;
	C_Home1.Data.Command=C_STOP;
	C_Home2.Data.Lv=0;
	C_Home2.Data.Rv=0;
	C_Home2.Data.Command=C_STOP;
	C_Home3.Data.Lv=0;
	C_Home3.Data.Rv=0;
	C_Home3.Data.Command=C_STOP;
	C_Home4.Data.Lv=0;
	C_Home4.Data.Rv=0;
	C_Home4.Data.Command=C_STOP;
	C_Home5.Data.Lv=0;
	C_Home5.Data.Rv=0;
	C_Home5.Data.Command=C_STOP;
	C_Home6.Data.Lv=0;
	C_Home6.Data.Rv=0;
	C_Home6.Data.Command=C_STOP;
	C_Home7.Data.Lv=0;
	C_Home7.Data.Rv=0;
	C_Home7.Data.Command=C_STOP;
	C_Home8.Data.Lv=0;
	C_Home8.Data.Rv=0;
	C_Home8.Data.Command=C_STOP;
	C_Home9.Data.Lv=0;
	C_Home9.Data.Rv=0;
	C_Home9.Data.Command=C_STOP;
	C_Home10.Data.Lv=0;
	C_Home10.Data.Rv=0;
	C_Home10.Data.Command=C_STOP;
	C_Home11.Data.Lv=0;
	C_Home11.Data.Rv=0;
	C_Home11.Data.Command=C_STOP;*/
	
	m_nStrategy = id;//left=1,right=0
	nKick = 0;
	nKick2 = 0;
	nShoot = 0;
	nSweep = 0;
}


CStrategySystem::~CStrategySystem() {

}

void CStrategySystem::Action() {
	Think();
}

void CStrategySystem::Think() {
	NormalGame5();
}


void CStrategySystem::NormalGame5() {
	int i;
	i = C_CheckBallPos();
	Position(HOME1, ball.position);
}


void CStrategySystem::Angle(int which, int desired_angle) {
	OurRobot *robot;
	int theta_e, vL, vR;
	
	switch (which) {
		case HOME1:
			robot = &home1;
			break;
		case HOME2:
			robot = &home2;
			break;
		case HOME3:
			robot = &home3;
			break;
		case HOME4:
			robot = &home4;
			break;
		case HOME5:
			robot = &home5;
			break;
		case HOME6:
			robot = &home6;
			break;
		case HOME7:
			robot = &home7;
			break;
		case HOME8:
			robot = &home8;
			break;
		case HOME9:
			robot = &home9;
			break;
		case HOME10:
			robot = &home10;
			break;
		case HGOALIE:
			robot = &hgoalie;
			break;
	}
	
	//����Ŀ���������˷���ǵĲ�
	theta_e = desired_angle - robot->angle;
	
	//�Ƕȹ淶���ڣ�-360��360��
	while (theta_e > 180)
		theta_e -= 360;
	while (theta_e < -180)
		theta_e += 360;
	
	//����ת�ĽǶȴ���90��С��-90����ת�䲹�ǣ����ı�������
	if (theta_e < -90)
		theta_e += 180;
	else if (theta_e > 90)
		theta_e -= 180;
	
	//����ϵ�������ȷ���ģ�
	vL = (int) (60.0 / 90.0 * theta_e);
	vR = (int) (-60.0 / 90.0 * theta_e);
	Velocity(which, vL, vR);
}

void CStrategySystem::Velocity(int which, int vL, int vR) {
	//��������Χ
	if (vL < -127) vL = -127;
	if (vL > 127) vL = 127;
	
	if (vR < -127) vR = -127;
	if (vR > 127) vR = 127;
	
	
	switch (which) {
		case HOME1:
			command[2] = vL;
			command[3] = vR;
			command[4] = C_GO;
			break;
		case HOME2:
			command[5] = vL;
			command[6] = vR;
			command[7] = C_GO;
			break;
		case HOME3:
			command[8] = vL;
			command[9] = vR;
			command[10] = C_GO;
			break;
		case HOME4:
			command[11] = vL;
			command[12] = vR;
			command[13] = C_GO;
			break;
		case HOME5:
			command[14] = vL;
			command[15] = vR;
			command[16] = C_GO;
			break;
		case HOME6:
			command[17] = vL;
			command[18] = vR;
			command[19] = C_GO;
			break;
		case HOME7:
			command[20] = vL;
			command[21] = vR;
			command[22] = C_GO;
			break;
		case HOME8:
			command[23] = vL;
			command[24] = vR;
			command[25] = C_GO;
			break;
		case HOME9:
			command[26] = vL;
			command[27] = vR;
			command[28] = C_GO;
			break;
		case HOME10:
			command[29] = vL;
			command[30] = vR;
			command[31] = C_GO;
			break;
		case HGOALIE:
			command[32] = vL;
			command[33] = vR;
			command[34] = C_GO;
			break;
	}
	
}


void CStrategySystem::Position(int which, CPoint point) {
	OurRobot *robot;
	double distance_e;
	int dx, dy, desired_angle, theta_e, vL, vR;
	
	switch (which) {
		case HOME1:
			robot = &home1;
			break;
		case HOME2:
			robot = &home2;
			break;
		case HOME3:
			robot = &home3;
			break;
		case HOME4:
			robot = &home4;
			break;
		case HOME5:
			robot = &home5;
			break;
		case HOME6:
			robot = &home6;
			break;
		case HOME7:
			robot = &home7;
			break;
		case HOME8:
			robot = &home8;
			break;
		case HOME9:
			robot = &home9;
			break;
		case HOME10:
			robot = &home10;
			break;
		case HGOALIE:
			robot = &hgoalie;
			break;
	}
	
	dx = point.x - robot->position.x;
	dy = point.y - robot->position.y;
	
	distance_e = sqrt(1.0 * dx * dx + 1.0 * dy * dy);
	
	if (dx == 0 && dy == 0)
		desired_angle = 90;
	else
		desired_angle = (int) (180.0 / M_PI * atan2((double) (dy), (double) (dx)));
	
	theta_e = desired_angle - robot->angle;
	
	while (theta_e > 180)
		theta_e -= 360;
	while (theta_e < -180)
		theta_e += 360;
	
	//���н��ڣ�90��180����-180��-90��֮�䣬��ת�䲹��
	//���˼�����ľ���
	if (theta_e < -90) {
		theta_e += 180;
		distance_e = -distance_e;
	} else if (theta_e > 90) {
		theta_e -= 180;
		distance_e = -distance_e;
	}
	
	
	vL = (int) (2. * (100.0 / 1000.0 * distance_e + 40.0 / 90.0 * theta_e));
	vR = (int) (2. * (100.0 / 1000.0 * distance_e - 40.0 / 90.0 * theta_e));


//	TRACE("\nin strategy vL=%d,vR=%d",vL,vR);
	Velocity(which, vL, vR);
}


#define G_OFFSET        20//15
#define G_ANGLE_BOUND    60
#define G_BOUND_BOUND    10

void CStrategySystem::Goalie(int which) {
	CPoint point;
	OurRobot *robot;
	int estimate_x, estimate_y;//, angle;
	int height, center;
	
	switch (which) {
		case HOME1:
			robot = &home1;
			break;
		case HOME2:
			robot = &home2;
			break;
		case HGOALIE:
			robot = &hgoalie;
			break;
	}
	
	height = boundRect.bottom - boundRect.top;
	center = (boundRect.bottom + boundRect.top) / 2;
	
	/*if(boundRect.right-ball.position.x  >= 60*3){
		//the Ball is far from the gate
		estimate_x = boundRect.right-G_OFFSET;
		   estimate_y = ball.position.y;
	   }
	else if(boundRect.right-ball.position.x  >= 15*3){
		//the Ball is in middle distance
		estimate_x = boundRect.right-G_OFFSET;
		estimate_y = ball.position.y;
	}
	else{
		//�����봦��ֱ�ӱ�����ӦΪ����棬�������׵����ţ�
		estimate_x = boundRect.right-G_OFFSET;
		   estimate_y = ball.position.y;
	}*/
	if (boundRect.right - ball.position.x >= 20 * 3) {
		estimate_x = boundRect.right - G_OFFSET;
		estimate_y = ball.position.y;
	} else {
		estimate_x = boundRect.right - G_OFFSET;
		CPoint position;
		position = C_NearestOppFromBall();
		double K, b;
		K = (position.y - ball.position.y) / (position.x - ball.position.x);
		b = position.y - K * position.x;
		estimate_y = (int) (K * estimate_x + b);
	}
	//��������������������޶�������ȥ������
	if (estimate_y < boundRect.top + height * 1 / 3)
		estimate_y = boundRect.top + height * 1 / 3;
	else if (estimate_y > boundRect.bottom - height * 1 / 3)
		estimate_y = boundRect.bottom - height * 1 / 3;
	
	GoaliePosition(which, CPoint(estimate_x, estimate_y));
}

void CStrategySystem::ReceiveData(Ball bal, OurRobot ho1, OurRobot ho2, OurRobot ho3, OurRobot ho4,
                                  OurRobot ho5, OurRobot ho6, OurRobot ho7, OurRobot ho8, OurRobot ho9,
                                  OurRobot ho10, OurRobot hgo, Opponent opp) {
	//�ҷ����Ұ�����ֱ�ӽ���
	if (m_nGameArea == GAME_RIGHT) {
		ball.position = bal.position;         //��
		
		home1.position = ho1.position;        //HOME1
		home1.angle = ho1.angle;
		
		home2.position = ho2.position;        //HOME2
		home2.angle = ho2.angle;
		
		home3.position = ho3.position;
		home3.angle = ho3.angle;
		
		home4.position = ho4.position;
		home4.angle = ho4.angle;
		
		home5.position = ho5.position;
		home5.angle = ho5.angle;
		
		home6.position = ho6.position;
		home6.angle = ho6.angle;
		
		home7.position = ho7.position;
		home7.angle = ho7.angle;
		
		home8.position = ho8.position;
		home8.angle = ho8.angle;
		
		home9.position = ho9.position;
		home9.angle = ho9.angle;
		
		home10.position = ho10.position;
		home10.angle = ho10.angle;
		
		hgoalie.position = hgo.position;      //HGOALIE
		hgoalie.angle = hgo.angle;
		
		opponent.position1 = opp.position1;   //OPPONENT
		opponent.position2 = opp.position2;
		opponent.position3 = opp.position3;
		opponent.position4 = opp.position4;
		opponent.position5 = opp.position5;
		opponent.position6 = opp.position6;
		opponent.position7 = opp.position7;
		opponent.position8 = opp.position8;
		opponent.position9 = opp.position9;
		opponent.position10 = opp.position10;
		opponent.position11 = opp.position11;
		
	} else  //���ҷ������������������ʱ����Ӧ�任
	{
		//����Ա��������Ϣλ�ù��ڳ������ĶԳ�
		ball.position.x = 1032 - bal.position.x;
		ball.position.y = 818 - bal.position.y;
		
		home1.position.x = 1032 - ho1.position.x;
		home1.position.y = 818 - ho1.position.y;
		///*
		if (home1.angle > 0)
			home1.angle = ho1.angle - 180;
		else
			home1.angle = 180 + ho1.angle;
		//*/
		home2.position.x = 1032 - ho2.position.x;
		home2.position.y = 818 - ho2.position.y;
		///*
		if (home2.angle > 0)
			home2.angle = ho2.angle - 180;
		else
			home2.angle = 180 + ho2.angle;
		//*/
		home3.position.x = 1032 - ho3.position.x;
		home3.position.y = 818 - ho3.position.y;
		///*
		if (home3.angle > 0)
			home3.angle = ho3.angle - 180;
		else
			home3.angle = 180 + ho3.angle;
		//*/
		home4.position.x = 1032 - ho4.position.x;
		home4.position.y = 818 - ho4.position.y;
		///*
		if (home4.angle > 0)
			home4.angle = ho4.angle - 180;
		else
			home4.angle = 180 + ho4.angle;
		//*/
		home5.position.x = 1032 - ho5.position.x;
		home5.position.y = 818 - ho5.position.y;
		///*
		if (home5.angle > 0)
			home5.angle = ho5.angle - 180;
		else
			home5.angle = 180 + ho5.angle;
		//*/
		home6.position.x = 1032 - ho6.position.x;
		home6.position.y = 818 - ho6.position.y;
		///*
		if (home6.angle > 0)
			home6.angle = ho6.angle - 180;
		else
			home6.angle = 180 + ho6.angle;
		//*/
		home7.position.x = 1032 - ho7.position.x;
		home7.position.y = 818 - ho7.position.y;
		///*
		if (home7.angle > 0)
			home7.angle = ho7.angle - 180;
		else
			home7.angle = 180 + ho7.angle;
		//*/
		home8.position.x = 1032 - ho8.position.x;
		home8.position.y = 818 - ho8.position.y;
		///*
		if (home8.angle > 0)
			home8.angle = ho8.angle - 180;
		else
			home8.angle = 180 + ho8.angle;
		//*/
		home9.position.x = 1032 - ho9.position.x;
		home9.position.y = 818 - ho9.position.y;
		///*
		if (home9.angle > 0)
			home9.angle = ho9.angle - 180;
		else
			home9.angle = 180 + ho9.angle;
		//*/
		home10.position.x = 1032 - ho10.position.x;
		home10.position.y = 818 - ho10.position.y;
		///*
		if (home10.angle > 0)
			home10.angle = ho10.angle - 180;
		else
			home10.angle = 180 + ho10.angle;
		//*/
		hgoalie.position.x = 1032 - hgo.position.x;
		hgoalie.position.y = 818 - hgo.position.y;
		///*
		if (hgoalie.angle > 0)
			hgoalie.angle = hgo.angle - 180;
		else
			hgoalie.angle = 180 + hgo.angle;
		//*/
		opponent.position1.x = 1032 - opp.position1.x;
		opponent.position2.x = 1032 - opp.position2.x;
		opponent.position3.x = 1032 - opp.position3.x;
		opponent.position4.x = 1032 - opp.position4.x;
		opponent.position5.x = 1032 - opp.position5.x;
		opponent.position6.x = 1032 - opp.position6.x;
		opponent.position7.x = 1032 - opp.position7.x;
		opponent.position8.x = 1032 - opp.position8.x;
		opponent.position9.x = 1032 - opp.position9.x;
		opponent.position10.x = 1032 - opp.position10.x;
		opponent.position11.x = 1032 - opp.position11.x;
		
		opponent.position1.y = 818 - opp.position1.y;
		opponent.position2.y = 818 - opp.position2.y;
		opponent.position3.y = 818 - opp.position3.y;
		opponent.position4.y = 818 - opp.position4.y;
		opponent.position5.y = 818 - opp.position5.y;
		opponent.position6.y = 818 - opp.position6.y;
		opponent.position7.y = 818 - opp.position7.y;
		opponent.position8.y = 818 - opp.position8.y;
		opponent.position9.y = 818 - opp.position9.y;
		opponent.position10.y = 818 - opp.position10.y;
		opponent.position11.y = 818 - opp.position11.y;
		
	}
}


void CStrategySystem::Stop(int which) {
	int vL, vR;
	vL = vR = 0;
	
	Velocity(which, vL, vR);
}

void CStrategySystem::GoaliePosition(int which, CPoint point) {
	OurRobot *robot;
	double distance_e;
	int dx, dy, desired_angle, theta_e, vL, vR;
	
	switch (which) {
		case HOME1:
			robot = &home1;
			break;
		case HOME2:
			robot = &home2;
			break;
		case HGOALIE:
			robot = &hgoalie;
			break;
	}

//	TRACE("\nin strategy yh1.x=%d,yh1.y=%d",robot->position.x,robot->position.y);
//	TRACE("\nin strategy point.x=%d,point.y=%d",point.x,point.y);
	dx = point.x - robot->position.x;
	dy = point.y - robot->position.y;
	
	distance_e = sqrt(1.0 * dx * dx + 1.0 * dy * dy);
	
	if (dx == 0 && dy == 0)
		desired_angle = 90;
	else
		desired_angle = (int) (180.0 / M_PI * atan2((double) (dy), (double) (dx)));
	
	theta_e = desired_angle - robot->angle;
	
	while (theta_e > 180)
		theta_e -= 360;
	while (theta_e < -180)
		theta_e += 360;
	
	//���н��ڣ�90��180����-180��-90��֮�䣬��ת�䲹��
	//���˼�����ľ���
	if (theta_e < -90) {
		theta_e += 180;
		distance_e = -distance_e;
	} else if (theta_e > 90) {
		theta_e -= 180;
		distance_e = -distance_e;
	}
	
	
	vL = (int) (20. * (100.0 / 400.0 * distance_e + 40.0 / 90.0 * theta_e));
	vR = (int) (20. * (100.0 / 400.0 * distance_e - 40.0 / 90.0 * theta_e));


//	TRACE("\nin strategy vL=%d,vR=%d",vL,vR);
	Velocity(which, vL, vR);
}

int CStrategySystem::C_CheckBallPos() {
	if (ball.position.x < (boundRect.left + (boundRect.right - boundRect.left) / 4)) {
		if (ball.position.y < (boundRect.top + (boundRect.bottom - boundRect.top) / 2))
			return 1;
		else
			return 2;
	} else if (ball.position.x > (boundRect.left + (boundRect.right - boundRect.left) / 4) &&
	           ball.position.x < (boundRect.left + (boundRect.right - boundRect.left) / 2)) {
		if (ball.position.y < (boundRect.top + (boundRect.bottom - boundRect.top) / 2))
			return 3;
		else
			return 4;
	} else if (ball.position.x > (boundRect.left + (boundRect.right - boundRect.left) / 2) &&
	           ball.position.x < (boundRect.left + 3 * (boundRect.right - boundRect.left) / 4)) {
		if (ball.position.y < (boundRect.top + (boundRect.bottom - boundRect.top) / 2))
			return 5;
		else
			return 6;
	} else {
		if (ball.position.y < (boundRect.top + (boundRect.bottom - boundRect.top) / 2))
			return 7;
		else
			return 8;
	}
	
}

void CStrategySystem::C_KickTo(int which, CPoint pos) {
	OurRobot *robot;
	CPoint target, mball;
	int dx, dy, theta_d, alpha;
	
	switch (which) {
		case HOME1:
			robot = &home1;
			break;
		case HOME2:
			robot = &home2;
			break;
		case HOME3:
			robot = &home3;
			break;
		case HOME4:
			robot = &home4;
			break;
		case HOME5:
			robot = &home5;
			break;
		case HOME6:
			robot = &home6;
			break;
		case HOME7:
			robot = &home7;
			break;
		case HOME8:
			robot = &home8;
			break;
		case HOME9:
			robot = &home9;
			break;
		case HOME10:
			robot = &home10;
			break;
		case HGOALIE:
			robot = &hgoalie;
			break;
	}
	
	mball.x = ball.position.x;
	mball.y = ball.position.y;
	
	switch (nKick) {
		case 0: //״̬0������һ��λ�ã��ܵ��õ�λ��ȥ����,��λ
			
			dx = mball.x - pos.x;
			dy = mball.y - pos.y;//(boundRect.top+boundRect.bottom)/2;
			target.x = mball.x + (int) (35.0 * dx / sqrt(dx * dx + dy * dy + 0.01));//ԭΪ25
			target.y = mball.y + (int) (35.0 * dy / sqrt(dx * dx + dy * dy + 0.01));//ԭΪ25
			Position(which, target);  //�ܵ��õ�λ��
			
			dx = robot->position.x - target.x;
			dy = robot->position.y - target.y;
			
			//�ܵ��õ�λ�ã���Ŀ���뾶Ϊ5��Բ��
			if (dx * dx + dy * dy < 25)
				nKick = 1;       //ת��״̬1
			break;
		
		case 1: //״̬1��������ת��һ���ĽǶ�׼������
			
			dx = mball.x - robot->position.x;  //dx<0
			dy = mball.y - robot->position.y;  //dyû����
			if (dx == 0 && dy == 0)
				alpha = 90;
			else
				alpha = (int) (180.0 / M_PI * atan2((double) dy, (double) dx));
			alpha -= robot->angle;
			while (alpha > 180)
				alpha -= 360;
			while (alpha < -180)
				alpha += 360;
			alpha = abs(alpha);
			//����н���Ԥ����Χ��
			if (alpha < 15 || alpha > 180 - 15) {//ԭΪ15,���Ϊ8,���Ϊ20
				nKick = 2;    //ת��״̬2��׼������
				//ShootVar������λ�ã��ǳ�Ա����
				ShootVar.x = ball.position.x;
				ShootVar.y = ball.position.y;
				//ShootLen���������ľ���
				ShootLen = leng(ShootVar.x, ShootVar.y, robot->position.x, robot->position.y);
			}
			dx = mball.x - pos.x;
			dy = mball.y - pos.y;//(boundRect.top+boundRect.bottom)/2;
			if (dx == 0 && dy == 0)
				theta_d = 90;
			else
				theta_d = (int) (180.0 / M_PI * atan2((double) (-dy), (double) (-dx)));
			Angle(which, theta_d);   //what if theta_d>180
			/*
			//���������ܵ������ߣ��ܹ���ͷ��
			if(robot->position.x<mball.x)
				nKick=6;    //ת��״̬6
			*/
			break;
		case 2: //״̬2������,shooting the ball
			
			dx = mball.x - robot->position.x;
			dy = mball.y - robot->position.y;
			if (dx == 0 && dy == 0)
				alpha = 90;
			else
				alpha = (int) (180.0 / M_PI * atan2((double) dy, (double) dx));
			alpha -= robot->angle;
			while (alpha > 180)
				alpha -= 360;
			while (alpha < -180)
				alpha += 360;
			alpha = abs(alpha);
			//shooting
			if (alpha < 90)
				Velocity(which, 10, 10);//110
			else
				Velocity(which, -12, -12);
			//���������������֮ǰ��ֵ��ת2����ֹͣ
			if (ShootLen < leng(ShootVar.x, ShootVar.y, robot->position.x, robot->position.y)) {
				
				nKick = 3;
			} else  //���¾��룬��һ��ʱ������
				ShootLen = leng(ShootVar.x, ShootVar.y, robot->position.x, robot->position.y);
			break;
		case 3:  //״̬3��ֹͣ
			
			Velocity(which, 0, 0);
			break;
	}
}


CPoint CStrategySystem::C_NearestOppFromBall() {
	int minidistance, distance;
	CPoint position;
	minidistance = CountDistance(opponent.position1, ball.position);
	position = opponent.position1;
	distance = CountDistance(opponent.position2, ball.position);
	if (distance < minidistance) {
		minidistance = distance;
		position = opponent.position2;
	}
	distance = CountDistance(opponent.position3, ball.position);
	if (distance < minidistance) {
		minidistance = distance;
		position = opponent.position2;
	}
	distance = CountDistance(opponent.position4, ball.position);
	if (distance < minidistance) {
		minidistance = distance;
		position = opponent.position2;
	}
	distance = CountDistance(opponent.position5, ball.position);
	if (distance < minidistance) {
		minidistance = distance;
		position = opponent.position2;
	}
	distance = CountDistance(opponent.position6, ball.position);
	if (distance < minidistance) {
		minidistance = distance;
		position = opponent.position2;
	}
	distance = CountDistance(opponent.position7, ball.position);
	if (distance < minidistance) {
		minidistance = distance;
		position = opponent.position2;
	}
	distance = CountDistance(opponent.position8, ball.position);
	if (distance < minidistance) {
		minidistance = distance;
		position = opponent.position2;
	}
	distance = CountDistance(opponent.position9, ball.position);
	if (distance < minidistance) {
		minidistance = distance;
		position = opponent.position2;
	}
	distance = CountDistance(opponent.position10, ball.position);
	if (distance < minidistance) {
		minidistance = distance;
		position = opponent.position2;
	}
	distance = CountDistance(opponent.position11, ball.position);
	if (distance < minidistance) {
		minidistance = distance;
		position = opponent.position2;
	}
	return position;
}

int CStrategySystem::CountDistance(CPoint point1, CPoint point2) {
	int distance;
	distance = (int) sqrt(pow(abs(point1.x - point2.x), 2) + pow(abs(point1.y - point2.y), 2));
	return distance;
}

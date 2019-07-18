// stdafx.cpp : source file that includes just the standard includes
//	MicroClient.pch will be the pre-compiled header
//	stdafx.obj will contain the pre-compiled type information

#include "stdafx.h"
#include "StrategySystem.h"

//�ѽ��յ��Ķ�Ա������ͽǶȷ���r[0][i]���棬�Է���Ա������
//����op����
//ȡÿ����Ա�����٣���Velocity�������������Ѿ��������
//����Ա������

/**********************************************************/
//nKick:������������˵Ķ���
//      0:	״̬0������һ��λ�ã��ܵ��õ�λ��ȥ����,��λ
//		1:  ״̬1��������ת��һ���ĽǶ�׼������
//		2:	״̬2������,shooting the ball
//		3:	״̬3��ֹͣ
/*********************************************************/

int nKick;
CStrategySystem *thePlannerR = new CStrategySystem(0);
CStrategySystem *thePlannerL = new CStrategySystem(1);
OurRobot r[2][11];
Opponent op;//��ŶԷ���Ա����������
Ball theball;//��������������.

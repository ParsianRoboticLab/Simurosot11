// stdafx.cpp : source file that includes just the standard includes
//	MicroClient.pch will be the pre-compiled header
//	stdafx.obj will contain the pre-compiled type information

#include "stdafx.h"
#include "StrategySystem.h"

//把接收到的队员的坐标和角度放入r[0][i]里面，对方队员的坐标
//放入op里面
//取每个队员的轮速，在Velocity函数计算里面已经获得了三
//个队员的轮速

/**********************************************************/
//nKick:代表各个机器人的动作
//      0:	状态0：处在一般位置，跑到好的位置去射门,定位
//		1:  状态1：机器人转动一定的角度准备射门
//		2:	状态2：射门,shooting the ball
//		3:	状态3：停止
/*********************************************************/

int nKick;
CStrategySystem* thePlannerR=new CStrategySystem(0);
CStrategySystem* thePlannerL=new CStrategySystem(1);
OurRobot r[2][11];
Opponent op;//存放对方队员的坐标数据
Ball theball;//存放球的坐标数据.

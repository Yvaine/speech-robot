/*************************************************************************
	> File Name: arduino_cmd.h
	> Author: lengjing
	> Mail: jingleng77@163.com 
	> Created Time: Thu Nov 19 17:44:19 2015
 ************************************************************************/
#include "arduino_cmd.h"
#include<iostream>
using namespace std;

const int back_limit = 120;
const int forward_limit = 100 ;
const int rotate_limit = 40;

char mkp_x(int v)
{
	char a = (abs(v) < 50) ? '0' : v > 0 ? '+' : '-';
	return a;
}
char mkp_y(int v)
{
	char a = (abs(v) < 50) ? '0' : v > 0 ? '+' : '-';
	return a;
}

char mk_motor(int x, int face_width)
{
	char cmd = 'q';
	if(abs(x) < rotate_limit)
	{
		if(face_width >= back_limit)
			cmd = 's';
		else if(face_width <= forward_limit)
			cmd = 'w';
		else
			cmd = 'q';
	}
	else
		cmd = x > 0 ? 'a' : 'd';
	return cmd;
}

char mk_motor2(int x, int y)
{
	char cmd = 'q';
	if(abs(x) < 80)
	{
		cmd = abs(y) < 50 ? 'q' : y < 0 ? 's' : 'w';
	}
	else
		cmd = x > 0 ? 'a' : 'd';
	return cmd;
}

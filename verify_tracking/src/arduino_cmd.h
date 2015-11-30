/*************************************************************************
	> File Name: arduino_cmd.h
	> Author: lengjing
	> Mail: jingleng77@163.com 
	> Created Time: Thu Nov 19 17:44:19 2015
 ************************************************************************/
#ifndef _ARDUINO_CMD_H
#define _ARDUINO_CMD_H

#include<iostream>
#include "stdlib.h"
#include "stdio.h"
using namespace std;

char mkp_x(int v);

char mkp_y(int v);

char mk_motor(int x, int face_width);

char mk_motor2(int x, int y);

#endif

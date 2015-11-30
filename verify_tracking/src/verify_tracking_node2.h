/*************************************************************************
	> File Name: verify_tracking_node2.h
	> Author: lengjing
	> Mail: jingleng77@163.com 
	> Created Time: Thu Nov 19 17:40:47 2015
 ************************************************************************/
#ifndef _VERIFY_TRACKING_NODE2_H
#define _VERIFY_TRACKING_NODE2_H

#include<iostream>
#include "arduino_cmd.h"
#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "std_msgs/String.h"

#include <opencv2/opencv.hpp>
#include <cv_face.h>
using namespace std;
using namespace ros;
using namespace cv;

int create_verify_feature_image(cv_handle_t &handle_detect, cv_handle_t &handle_verify, const Mat &p_image_color, cv_feature_t *p_feature_new, int frame_width, int frame_height);

int create_verify_feature_db(cv_handle_t &handle_detect, cv_handle_t &handle_verify, int person_num, cv_feature_t **p_feature_new_1, int frame_width, int frame_height);


int verify_from_camera( cv_handle_t &handle_detect, cv_handle_t &handle_verify, VideoCapture &capture, cv_feature_t **p_feature_new_1, int person_number, int frame_width, int frame_height);

void max_func(const float *score, const int person_number, float &max_score, int &max_id);

int face_track(cv_handle_t &handle_track, VideoCapture &capture, const Point &expect, const int frame_width,  const int frame_heigh, Publisher &servox_pub, Publisher &servoy_pub, Publisher &motor_pub);

#endif


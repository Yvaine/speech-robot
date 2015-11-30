/*************************************************************************
	> File Name: camera_tracking.cpp
	> Author: lengjing
	> Mail: jingleng77@163.com 
	> Created Time: Fri Oct 23 14:30:01 2015
 ************************************************************************/

#include<iostream>
#include <vector>
#include <stdio.h>
#include <cv_face.h>

#include <opencv2/opencv.hpp>

#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Char.h"
#include <sys/time.h>

using namespace std;
using namespace cv;

char rotate_cmd[2];
char motor_cmd = 'q';
std_msgs::Char servox;
std_msgs::Char servoy; 
std_msgs::Char motor; 

const int frame_width = 640;
const int frame_width_half = frame_width >> 1;
const int left_limit = frame_width_half - 30;
//const int back_limit = 0.75 * frame_width;
//const int forward_limit = 0.55 * frame_width;
const int back_limit = 40;
const int forward_limit = 40 ;

inline char mkp_x(int v)
{
	char a = (abs(v) < 50) ? '0' : v > 0 ? '+' : '-';
	return a;
}
inline char mkp_y(int v)
{
	char a = (abs(v) < 50) ? '0' : v > 0 ? '+' : '-';
	return a;
}

inline char mk_motor(int x, int face_width)
{
	char cmd = 'q';
	if(abs(x) < 40)
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

inline char mk_motor2(int x, int y)
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


#define COST_TIME(tstart, tend) 1000000 * (tend.tv_sec - tstart.tv_sec) + tend.tv_usec- tstart.tv_usec

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "controlservo");
	ros::NodeHandle n;
	ros::Publisher servox_pub = n.advertise<std_msgs::Char>("servox_chatter", 1000);
	ros::Publisher servoy_pub = n.advertise<std_msgs::Char>("servoy_chatter", 1000);
	ros::Publisher motor_pub = n.advertise<std_msgs::Char>("motor_chatter", 1000);

	std::string port;
	ros::param::param<std::string>("~port", port, "/dev/ttyACM0");
	int baud;
	ros::param::param<int>("~baud", baud, 57600);
	ros::Rate loop_rate(10);
	VideoCapture capture;
	capture.open(0);         // open the camera
	if (!capture.isOpened()) {
		fprintf(stderr, "Liveness can not open camera!\n");
		return -1;
	}

	cv_handle_t handle_track = NULL;
	cv_face_t *p_face = NULL;
	int face_count = 0;
	cv_result_t cv_result = CV_OK;
	Mat read_frame, temp_frame, color_frame;
	//namedWindow("TrackingTest");
	int frame_width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    int frame_half_width = frame_width >> 1;
	int frame_half_height = frame_height >> 1;
	//printf("width %d height %d \n", frame_width, frame_height);
	Point expect(frame_half_width , frame_half_height);
	// init handle
	handle_track = cv_face_create_tracker(NULL, CV_FACE_SKIP_BELOW_THRESHOLD);
	if (!handle_track) {
		fprintf(stderr, "fail to init track handle\n");
		goto RETURN;
	}

	while (capture.read(read_frame)) {
		read_frame.copyTo(temp_frame);
		resize(temp_frame, temp_frame, Size(frame_width, frame_height), 0, 0,
		       INTER_LINEAR);
		cvtColor(temp_frame, color_frame, CV_BGR2BGRA);
		// realtime track
		face_count = 0;
#ifdef TIME
		struct timeval start, end;
		gettimeofday(&start, NULL);
#endif
		cv_result = cv_face_track(handle_track, color_frame.data, CV_PIX_FMT_BGRA8888,
						color_frame.cols, color_frame.rows, color_frame.cols,
						CV_FACE_UP, &p_face, &face_count);
#ifdef TIME
		gettimeofday(&end, NULL);
		double time = COST_TIME(start, end);
		printf("time cost = %.2f \n", time / 1000);
#endif
		if (cv_result != CV_OK) {
			fprintf(stderr, "cv_face_multi_track error : %d\n", cv_result);
			goto RETURN;
		}
#if 0
		if(p_face)
			printf("have face \n");
		else
			printf("no face\n");
#endif
		if(p_face){
			int face_width = p_face[0].rect.right - p_face[0].rect.left;
			int face_height = p_face[0].rect.bottom - p_face[0].rect.top;
			int face_width_half = face_width >> 1;
			int face_height_half = face_height >> 1;
		Point center(p_face[0].rect.left + face_width_half, p_face[0].rect.top + face_height_half);
	    Point target( -(center.x - expect.x), -(expect.y - center.y));
#if 1
		rotate_cmd[0] = mkp_x(target.x);
		printf(" target.x = %d target.y =%d motor_cmd = %c \n", target.x ,target.y, motor_cmd);
		rotate_cmd[1] = mkp_y(target.y);
		//printf(" center.y = %d target.y = %d rotate_cmd[1] = %c\n", center.y ,target.y, rotate_cmd[1]);
		motor_cmd = mk_motor2(target.x, target.y);
		servox.data = rotate_cmd[0];
		servoy.data = rotate_cmd[1];
		motor.data = motor_cmd;
		//servox_pub.publish(servox);
		//servoy_pub.publish(servoy);
		motor_pub.publish(motor);
#endif
#if 0
		for (int i = 0; i < face_count; i++) {
			fprintf(stderr, "face: %d-----[%d, %d, %d, %d]-----id: %d\n", i,
				p_face[i].rect.left, p_face[i].rect.top,
				p_face[i].rect.right, p_face[i].rect.bottom, p_face[i].ID);

			// draw the vedio
			Scalar scalar_color = CV_RGB(p_face[i].ID * 53 % 256,
						p_face[i].ID * 93 % 256,
						p_face[i].ID * 143 % 256);
			rectangle(temp_frame, Point2f(static_cast<float>(p_face[i].rect.left),
						static_cast<float>(p_face[i].rect.top)),
				  Point2f(static_cast<float>(p_face[i].rect.right),
					  static_cast<float>(p_face[i].rect.bottom)), scalar_color, 2);
			for (int j = 0; j < 21; j++) {
				circle(temp_frame, Point2f(p_face[i].points_array[j].x,
							p_face[i].points_array[j].y), 3, Scalar(0, 255, 0));
			}
		}
#endif
		//loop_rate.sleep();

		// release the memory of face
		cv_face_release_tracker_result(p_face, face_count);
		}
		//imshow("TrackingTest", temp_frame);
		if (waitKey(5) == 27 )
			break;
	}

RETURN:
	// destroy track handle
	cv_face_destroy_tracker(handle_track);
}



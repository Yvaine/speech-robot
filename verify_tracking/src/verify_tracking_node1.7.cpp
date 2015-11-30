/*************************************************************************
	> File Name: verify_tracking_node1.3.cpp
	> Author: lengjing
	> Mail: jingleng77@163.com 
	> Created Time: Mon Nov 23 16:43:14 2015
 ************************************************************************/
/*
 * Add verify msg to voice node. 2015.11.20
 * Add tracking recv voice cmd. 2015.11.20
 * Fixed problem of break of loop. 2015.11.23
 * Fixed can not find the face.  2015.11.24
 * Add host verify. 2015.11.26
 * Add ignore of image  verfigy. 2015.11.30
 */
#include<iostream>
//#include "time_helper.h"
#include <opencv2/opencv.hpp>
#include <cv_face.h>

#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "std_msgs/String.h"
#include <sys/time.h>
#include <vector>
#define DEFAULT_THRESHOLD (0.5)

//#define TIME
#define COST_TIME(tstart, tend)  (1000000 * (tend.tv_sec - tstart.tv_sec) + tend.tv_usec- tstart.tv_usec )
using namespace std;
using namespace cv;
using namespace ros;

enum catptureStatus{OPEN, READ, CLOSE, Verified, NOFACE}capStatus;

char rotate_cmd[2];
char motor_cmd = 'q';

std_msgs::Char servox;
std_msgs::Char servoy; 
std_msgs::Char motor; 

volatile char host_flag = '0';

volatile char verify_flag = '0';
std_msgs::Char verify;

volatile char track_flag = '0';
std_msgs::Char track;

const int frame_width = 640;
const int frame_width_half = frame_width >> 1;
const int left_limit = frame_width_half - 30;
//const int back_limit = 0.75 * frame_width;
//const int forward_limit = 0.55 * frame_width;
const int back_limit = 120;
const int forward_limit = 100 ;
const int rotate_limit = 40;

int face_track(VideoCapture &capture, Point &expect, int frame_width,  int frame_height, cv_handle_t &handle_track, ros::Publisher servox_pub, ros::Publisher servoy_pub ,ros::Publisher motor_pub);

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

void trackCallback(const std_msgs::Char &track_msg)
{
	if(track_msg.data == '0')
		track_flag = '0';
	else
		track_flag = '1';
}

void hostCallback(const std_msgs::Char &host_msg)
{
	if(host_msg.data == '0')
		host_flag = '0';
	else
		host_flag = '1';
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "verify_tracking_node");
	ros::NodeHandle n;
	std::string port;
	ros::param::param<std::string>("~port", port, "/dev/ttyACM0");
	int baud;
	ros::param::param<int>("~baud", baud, 57600);
	ros::Rate loop_rate(10);

	ros::Publisher servox_pub = n.advertise<std_msgs::Char>("servox_chatter", 1000);
	ros::Publisher servoy_pub = n.advertise<std_msgs::Char>("servoy_chatter", 1000);
	ros::Publisher motor_pub = n.advertise<std_msgs::Char>("motor_chatter", 1000);

	ros::Publisher verify_pub = n.advertise<std_msgs::Char>("verify_chatter", 1);

	Subscriber track_pub = n.subscribe("track_chatter", 1, trackCallback);
	Subscriber host_sub = n.subscribe("host_chatter", 1, hostCallback);
	
	cv_result_t cv_result = CV_OK;
	int main_return = -1;
	cv_handle_t handle_detect = NULL;
	cv_handle_t handle_track = NULL;
	VideoCapture capture;
	double time;
	capture.open(0);         // open the camera
	if (!capture.isOpened()) {
		fprintf(stderr, "Verify track can not open camera!\n");
		return -1;
	}
	capStatus = OPEN;
	int frame_width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    int frame_half_width = frame_width >> 1;
	int frame_half_height = frame_height >> 1;
	//printf("width %d height %d \n", frame_width, frame_height);
	Point expect(frame_half_width , frame_half_height);
	struct timeval start0, end0;
	struct timeval start1, end1;
	struct timeval start2, end2;
	struct timeval start3, end3;
	struct timeval start4, end4;
	struct timeval start5, end5;
#ifdef TIME
		gettimeofday(&start0, NULL);
#endif
	cv_handle_t handle_verify = cv_verify_create_handle("data/verify.tar");
#ifdef TIME
		gettimeofday(&end0, NULL);
		time = COST_TIME(start0, end0);
		printf("get from verify tar time cost = %.2fs \n", time / 1000000);
#endif
#if 1
	const int person_number = 5;
	Mat p_image_color_1[person_number], p_image_color_color_1[person_number], p_image_color_2, p_image_color_color_2;
	Mat tmp_frame;
	cv_face_t *p_face_1[person_number];
	cv_face_t *p_face_2;
	int face_count_1[person_number] = {0};
	int face_count_2 = 0;
	cv_feature_t *p_feature_1[person_number];
	cv_feature_t *p_feature_new_1[person_number];
	unsigned int feature_length_1[person_number];
	p_image_color_1[0] = imread("00.JPG");
	p_image_color_1[1] = imread("01.JPG");
	p_image_color_1[2] = imread("02.JPG");
	p_image_color_1[3] = imread("03.JPG");
	p_image_color_1[4] = imread("04.JPG");
	char *string_feature_1[person_number];
#else
	Mat p_image_color_2, p_image_color_color_2;

	const int person_number = 4;
	cv_face_t *p_face_2 = NULL;
	vector<cv_face_t *>p_face_1(person_number,NULL);
	vector<int>face_count_1(person_number, 0);
	int face_count_2 = 0;
	vector<Mat>p_image_color_1(person_number);
	vector<Mat>p_image_color_color_1(person_number);
	vector<cv_feature_t *>p_feature_1(person_number, NULL);
	vector<cv_feature_t *>p_feature_new_1(person_number, NULL);
	vector<unsigned int>feature_length_1(person_number, 0);
	// load image
	p_image_color_1.push_back(imread("01.JPG"));
	p_image_color_1.push_back(imread("02.JPG"));
	p_image_color_1.push_back(imread("03.JPG"));
	p_image_color_1.push_back(imread("04.JPG"));
	char *string_feature_1[person_number];
#endif

	for(int i = 0; i < person_number; i++)
	{
		if (!p_image_color_1[i].data ) {
			fprintf(stderr, "fail to read %d image \n", i);
			//return -1;
			goto RETURN;
		}
	}
	for(int i = 0; i < person_number; i++)
		cvtColor(p_image_color_1[i], p_image_color_color_1[i], CV_BGR2BGRA);
	// init detect handle
	handle_detect = cv_face_create_detector(NULL, CV_FACE_SKIP_BELOW_THRESHOLD | CV_DETECT_ENABLE_ALIGN);
	if (!handle_detect) {
		fprintf(stderr, "fail to init detect handle\n");
		goto RETURN;
		//return -1;
	}
	// detect
#ifdef TIME
		gettimeofday(&start1, NULL);
#endif
	for(int i = 0; i < person_number; i++)
		cv_result = cv_face_detect(handle_detect, p_image_color_color_1[i].data, CV_PIX_FMT_BGRA8888,
			p_image_color_color_1[i].cols, p_image_color_color_1[i].rows, p_image_color_color_1[i].step,
			CV_FACE_UP, &p_face_1[i], &face_count_1[i]);
#ifdef TIME
		gettimeofday(&end1, NULL);
		time = COST_TIME(start1, end1);
		printf("face detect from db time cost = %.2fs \n", time / 1000000);
#endif
	if (cv_result != CV_OK) {
		fprintf(stderr, "st_face_detect error : %d\n", cv_result);
		goto RETURN;
		//return -1;
	}
	for(int i = 0; i < person_number; i++)
	{
		if(face_count_1[i] == 0){
			fprintf(stderr, "can't find face in db %d", i);
			goto RETURN;
		}
	}
	if (handle_verify) {
#ifdef TIME
		gettimeofday(&start2, NULL);
#endif
		
	for(int i = 0; i < person_number; i++)
		cv_result = cv_verify_get_feature(handle_verify, p_image_color_color_1[i].data, CV_PIX_FMT_BGRA8888,
				p_image_color_color_1[i].cols,
				p_image_color_color_1[i].rows, p_image_color_color_1[i].step, p_face_1[i], &p_feature_1[i],
				&feature_length_1[i]);
#ifdef TIME
		gettimeofday(&end2, NULL);
		time = COST_TIME(start2, end2);
		printf("get feature from db time cost = %.2fs \n", time / 1000000);
#endif
	}
	else {
		fprintf(stderr, "fail to init verify handle, check for the model file!\n");
		goto RETURN;
	}
	for(int i = 0; i < person_number; i++)
	{
		if (feature_length_1[i] > 0) {
			cv_feature_header_t *p_feature_header = CV_FEATURE_HEADER(p_feature_1[i]);
			fprintf(stderr, "Feature information:\n");
			fprintf(stderr, "    ver:\t0x%08x\n", p_feature_header->ver);
			fprintf(stderr, "    length:\t%d bytes\n", p_feature_header->len);

			// test serial and deserial
			string_feature_1[i] = new char[CV_ENCODE_FEATURE_SIZE(p_feature_1[i])];
			cv_verify_serialize_feature(p_feature_1[i], string_feature_1[i]);
			p_feature_new_1[i] = cv_verify_deserialize_feature(string_feature_1[i]);
			delete []string_feature_1[i];
		}
		else {
			fprintf(stderr, "error, the feature length [%d]is 0!\n", i);
		}
	}
	handle_track = cv_face_create_tracker(NULL, CV_FACE_SKIP_BELOW_THRESHOLD);
	if (!handle_track) {
		fprintf(stderr, "fail to init track handle\n");
		goto RETURN;
	}
	//namedWindow("TrackingTest");
	//while (capture.read(p_image_color_2)) {
	
	while(capture.isOpened()) {
		for(int i = 0; i < 2; i++)
		{
			capture.read(tmp_frame);
		}
        tmp_frame.copyTo(p_image_color_2); 
		resize(p_image_color_2, p_image_color_2, Size(frame_width, frame_height), 0, 0, INTER_LINEAR);
		cvtColor(p_image_color_2, p_image_color_color_2, CV_BGR2BGRA);

#ifdef TIME
			gettimeofday(&start3, NULL);
#endif
			//printf("begin to detect from camera\n");
		cv_result = cv_face_detect(handle_detect, p_image_color_color_2.data, CV_PIX_FMT_BGRA8888,
				p_image_color_color_2.cols, p_image_color_color_2.rows, p_image_color_color_2.step,
				CV_FACE_UP, &p_face_2, &face_count_2);
#ifdef TIME
			gettimeofday(&end3, NULL);
			time = COST_TIME(start3, end3);
			printf("face detect from camera time cost = %.2fs \n", time / 1000000);
#endif
		if (cv_result != CV_OK) {
			fprintf(stderr, "st_face_detect error : %d\n", cv_result);
			goto RETURN;
		}
		spinOnce();

		if(host_flag == '0')
		{
			printf("host_flag = %c\n", host_flag);
			continue;
		}
		else
			printf("host_flag = %c\n", host_flag);

		// verify the first face
		if (face_count_2 > 0) {
			cv_feature_t *p_feature_2 = NULL;
			vector<float>score(person_number, 0);
			unsigned int feature_length_2;
			// get feature
			//printf("begin to get feature from camera\n");
#ifdef TIME
			gettimeofday(&start4, NULL);
#endif
			printf("begin to get feataure from camera\n");
			cv_result = cv_verify_get_feature(handle_verify, p_image_color_color_2.data, CV_PIX_FMT_BGRA8888,
					p_image_color_color_2.cols,
					p_image_color_color_2.rows, p_image_color_color_2.step, p_face_2, &p_feature_2,
					&feature_length_2);
#ifdef TIME
			gettimeofday(&end4, NULL);
			time = COST_TIME(start4, end4);
			printf("get feature from camera time cost = %.2fs \n", time / 1000000);
#endif

			if ( feature_length_2 > 0) {
				char *string_feature_2 = new char[CV_ENCODE_FEATURE_SIZE(p_feature_2)];
				cv_verify_serialize_feature(p_feature_2, string_feature_2);
				cv_feature_t *p_feature_new_2 = cv_verify_deserialize_feature(string_feature_2);
				delete []string_feature_2;

				// compare feature
#ifdef TIME
				gettimeofday(&start5, NULL);
#endif
				printf("begin to compare feature with db\n");
				for(int i = 0; i < person_number; i++)
				{
					cv_result = cv_verify_compare_feature(handle_verify, p_feature_new_1[i],
						p_feature_new_2, &score[i]);
				}
#ifdef TIME
				gettimeofday(&end5, NULL);
				time = COST_TIME(start5, end5);
				printf("compare feature time cost = %.2fms \n", time / 1000);
#endif
				if (cv_result == CV_OK) {
					float max_score = score[0];
					int max_id = 0;
					for(int i = 1; i < person_number; i++)
					{
						if(score[i] > max_score)
						{
							max_score = score[i];
							max_id = i;
						}
					}

					fprintf(stderr, "max score: %f\n", max_score);
					// comapre score with DEFAULT_THRESHOLD
					// > DEFAULT_THRESHOLD => the same person
					// < DEFAULT_THRESHOLD => different people
					if (max_score > DEFAULT_THRESHOLD)
					{
						fprintf(stderr, "you are the right person, your number is %d\n", max_id);
						capStatus = Verified;
						// send verify_flag msg to verify chatter
						verify_flag = '1';
						verify.data = verify_flag;
						verify_pub.publish(verify);
						//printf("verify node publish verify flag %c to speech node\n", verify_flag);
						spinOnce();
						printf("track flag %c\n", track_flag);
						if(track_flag == '0')
							continue;
						int track_value = face_track(capture, expect, frame_width, frame_height, handle_track, servox_pub, servoy_pub,motor_pub);
						if(track_value == -1)
						{
							printf("no face detected !, verified frome start!\n");
							verify_flag = '0';
							verify.data = verify_flag;
							verify_pub.publish(verify);
							track_flag = '0';
							host_flag = '0';
							continue;
						}
					}
					else
					{
						fprintf(stderr, "no you are not right person .\n");
						verify_flag = '0';
						verify.data = verify_flag;
						verify_pub.publish(verify);
						track_flag = '0';
						host_flag = '0';
						continue;
					}
				} else {
					fprintf(stderr, "cv_verify_compare_feature error : %d\n", cv_result);
				}
				cv_verify_release_feature(p_feature_new_2);
			} else {
				fprintf(stderr, "error, the feature length is 0!\n");
			}
			cv_verify_release_feature(p_feature_2);
		} else {
			fprintf(stderr, "no face in camera\n");
			verify_flag = '0';
			verify.data = verify_flag;
			verify_pub.publish(verify);
			track_flag = '0';
			host_flag = '0';
			printf("verify_flag = %c, host_flag = %c\n", verify_flag, host_flag);
			continue;
		}
#if 0
		Scalar scalar_color = CV_RGB(p_face[i].ID * 53 % 256,
				p_face[i].ID * 93 % 256,
				p_face[i].ID * 143 % 256);
		rectangle(temp_frame, Point2f(static_cast<float>(p_face[i].rect.left),
					static_cast<float>(p_face[i].rect.top)),
				Point2f(static_cast<float>(p_face[i].rect.right),
					static_cast<float>(p_face[i].rect.bottom)), scalar_color, 2);
#endif
#if 0
		imshow("TrackingTest", p_image_color_2);
		if (waitKey(5) == 27 )
			break;
#endif
		// release the memory of feature

	}
	for(int i = 1; i < person_number; i++)
	{
		cv_verify_release_feature(p_feature_new_1[i]);
		cv_verify_release_feature(p_feature_1[i]);
	}
	// destroy verify handle
	cv_verify_destroy_handle(handle_verify);
RETURN:
	// release the memory of face
	for(int i = 1; i < person_number; i++)
		cv_face_release_detector_result(p_face_1[i], face_count_1[i]);
	cv_face_release_detector_result(p_face_2, face_count_2);
	// destroy detect handle
	cv_face_destroy_detector(handle_detect);

	fprintf(stderr, "test finish!\n");
	return main_return;
}

int face_track(VideoCapture &capture, Point &expect, int frame_width,  int frame_height, cv_handle_t &handle_track,ros::Publisher servox_pub, ros::Publisher servoy_pub, ros::Publisher motor_pub)
{
	cv_face_t *p_face = NULL;
	int face_count = 0;
	cv_result_t cv_result = CV_OK;
	Mat temp_frame, color_frame;
    while(capture.read(temp_frame))
	{
		resize(temp_frame, temp_frame, Size(frame_width, frame_height), 0, 0,
		       INTER_LINEAR);
		cvtColor(temp_frame, color_frame, CV_BGR2BGRA);
		// realtime track
		cv_result = cv_face_track(handle_track, color_frame.data, CV_PIX_FMT_BGRA8888,
						color_frame.cols, color_frame.rows, color_frame.cols,
						CV_FACE_UP, &p_face, &face_count);
		if (cv_result != CV_OK) {
			fprintf(stderr, "cv_face_multi_track error : %d\n", cv_result);
			goto RETURN;
		}
		if(p_face){
			int face_width = p_face[0].rect.right - p_face[0].rect.left;
			int face_height = p_face[0].rect.bottom - p_face[0].rect.top;
			int face_width_half = face_width >> 1;
			int face_height_half = face_height >> 1;
		Point center(p_face[0].rect.left + face_width_half, p_face[0].rect.top + face_height_half);
	    Point target( -(center.x - expect.x), -(expect.y - center.y));

		rotate_cmd[0] = mkp_x(target.x);
		rotate_cmd[1] = mkp_y(target.y);
		printf(" face_width = %d target.x = %d target.y =%d motor_cmd = %c \n", face_width, target.x ,target.y, motor_cmd);
		//printf(" center.y = %d target.y = %d rotate_cmd[1] = %c\n", center.y ,target.y, rotate_cmd[1]);
		motor_cmd = mk_motor(target.x, face_width);
		//motor_cmd = mk_motor2(target.x, target.y);
		servox.data = rotate_cmd[0];
		servoy.data = rotate_cmd[1];
		motor.data = motor_cmd;
		servox_pub.publish(servox);
		servoy_pub.publish(servoy);
		//motor_pub.publish(motor);
		cv_face_release_tracker_result(p_face, face_count);
		}
		else{
			capStatus = NOFACE;
			motor.data = 'q';
			return -1;
		}
		//motor_pub.publish(motor);
	}
	return -1;
RETURN:
	// destroy track handle
	cv_face_destroy_tracker(handle_track);
}

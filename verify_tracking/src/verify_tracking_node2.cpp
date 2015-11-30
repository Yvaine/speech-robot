/*************************************************************************
	> File Name: verify_tracking_node2.cpp
	> Author: lengjing
	> Mail: jingleng77@163.com 
	> Created Time: Thu Nov 19 14:29:21 2015
 ************************************************************************/

#include<iostream>
#include "verify_tracking_node2.h"
//#include "time_helper.h"
#include <opencv2/opencv.hpp>
#include <cv_face.h>

#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "std_msgs/String.h"
#include "arduino_cmd.h"
#include <sys/time.h>
#include <vector>
#include <unistd.h>
#define DEFAULT_THRESHOLD (0.5)

using namespace std;
using namespace cv;
using namespace ros;

#define TIME
#define COST_TIME(tstart, tend)  (1000000 * (tend.tv_sec - tstart.tv_sec) + tend.tv_usec- tstart.tv_usec )

char rotate_cmd[2];
char motor_cmd = 'q';
std_msgs::Char servox;
std_msgs::Char servoy; 
std_msgs::Char motor; 

int main(int argc, char *argv[])
{
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

	const int person_number = 3;
	cv_result_t cv_result = CV_OK;
	cv_handle_t handle_detect = NULL;
	cv_handle_t handle_track = NULL;
	cv_handle_t handle_verify = NULL;
	cv_feature_t *p_feature_new_1[person_number];

	int main_return = -1;
	int verify_flag = 0;
	int face_detect_flag = 0;
	VideoCapture capture;
	capture.open(0);         // open the camera
	if (!capture.isOpened()) {
		fprintf(stderr, "Liveness can not open camera!\n");
		return -1;
	}
	int frame_width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    int frame_half_width = frame_width >> 1;
	int frame_half_height = frame_height >> 1;
	//printf("width %d height %d \n", frame_width, frame_height);
	Point expect(frame_half_width , frame_half_height);
	handle_verify = cv_verify_create_handle("data/verify.tar");
	if(!handle_verify){
		fprintf(stderr, "failed to init verify handle \n");
		goto RETURN;
	}
	handle_track = cv_face_create_tracker(NULL, CV_FACE_SKIP_BELOW_THRESHOLD);
	if (!handle_track) {
		fprintf(stderr, "fail to init track handle\n");
		goto RETURN;
	}
	handle_detect = cv_face_create_detector(NULL, CV_FACE_SKIP_BELOW_THRESHOLD | CV_DETECT_ENABLE_ALIGN);
	if (!handle_detect) {
		fprintf(stderr, "fail to init detect handle\n");
		//goto RETURN;
		return -1;
	}
	create_verify_feature_db(handle_detect, handle_verify, person_number, p_feature_new_1, frame_width, frame_height);
	while(1)
	{
		int verfiy_flag = verify_from_camera(handle_detect, handle_verify, capture, p_feature_new_1, person_number, frame_width, frame_height);
		if(verify_flag != 1)
			continue;
		face_track(handle_track, capture, expect, frame_width, frame_height, servox_pub, servoy_pub, motor_pub);
	}
	for(int i = 1; i < person_number; i++)
	{
		cv_verify_release_feature(p_feature_new_1[i]);
	}
	// destroy verify handle
RETURN:
	// release the memory of face
	cv_verify_destroy_handle(handle_verify);
	// destroy detect handle
	cv_face_destroy_detector(handle_detect);

	fprintf(stderr, "test finish!\n");
}
int create_verify_feature_image(cv_handle_t &handle_detect, cv_handle_t &handle_verify, const Mat &p_image_color, cv_feature_t *p_feature_new, int frame_width, int frame_height)
{
	Mat p_image_color_color;
	cv_face_t *p_face;
	int face_count = 0;
	cv_feature_t *p_feature = NULL;
	//p_image_color.copyTo(p_image_color);
	//resize(p_image_color_2, p_image_color_2, Size(frame_width, frame_height), 0, 0, INTER_LINEAR);
	cvtColor(p_image_color, p_image_color_color, CV_BGR2BGRA);
	cv_result_t cv_result = cv_face_detect(handle_detect, p_image_color_color.data, CV_PIX_FMT_BGRA8888,
			p_image_color_color.cols, p_image_color_color.rows, p_image_color_color.step,
			CV_FACE_UP, &p_face, &face_count);
	if (cv_result != CV_OK) {
		fprintf(stderr, "st_face_detect error : %d\n", cv_result);
		//goto RETURN;
		//break;
	}
	// verify the first face
	printf("face count : %d \n", face_count);
	if (face_count > 0)
	{
		unsigned int feature_length;
		// get feature
		cv_result = cv_verify_get_feature(handle_verify, p_image_color_color.data, CV_PIX_FMT_BGRA8888,
				p_image_color_color.cols,
				p_image_color_color.rows, p_image_color_color.step, p_face, &p_feature,
				&feature_length);
		if ( feature_length > 0) {
			cv_feature_header_t *p_feature_header = CV_FEATURE_HEADER(p_feature);
			fprintf(stderr, "Feature information:\n");
			fprintf(stderr, "    ver:\t0x%08x\n", p_feature_header->ver);
			fprintf(stderr, "    length:\t%d bytes\n", p_feature_header->len);
			char *string_feature = new char[CV_ENCODE_FEATURE_SIZE(p_feature)];
			cv_verify_serialize_feature(p_feature, string_feature);
			p_feature_new = cv_verify_deserialize_feature(string_feature);
			delete []string_feature;
		}
		else{
			fprintf(stderr, "error, the feature length is 0!\n");
		}
	}
	else
	{
		fprintf(stderr, "no face in image!\n");
		return -1;
	}
	cv_verify_release_feature(p_feature);
	cv_face_release_detector_result(p_face, face_count);
	return 0;
}

int create_verify_feature_db(cv_handle_t &handle_detect, cv_handle_t &handle_verify, int person_number, cv_feature_t **p_feature_new_1, int frame_width, int frame_height)
{
	Mat p_image_color_1[person_number];
	int create_verify_flag = 0;
	char filename[20];
	for(int i = 0 ; i < person_number; i++)
	{
		sprintf(filename, "%02d.JPG",i);
		p_image_color_1[i] = imread(filename);
		if (!p_image_color_1[i].data ) {
			fprintf(stderr, "fail to read %d image \n", i);
			return -1;
			//goto RETURN;
		}
		fprintf(stderr, "begin to get feature from db[%d]\n", i);
		create_verify_flag = create_verify_feature_image(handle_detect, handle_verify, p_image_color_1[i], p_feature_new_1[i], frame_width, frame_height);
		if(create_verify_flag == -1)
			fprintf(stderr, "failed to create_verify_feature_image %d", i);
	}
	return 0;
}

int verify_from_camera(cv_handle_t &handle_detect, cv_handle_t &handle_verify, VideoCapture &capture, cv_feature_t **p_feature_new_1, int person_number, int frame_width, int frame_height)
{
	Mat tmp_frame, tmp_frame2, tmp_frame3;
	cv_feature_t *p_feature_new_2;
	float score[person_number];
	float max_score = 0.0f;
	int max_id = 0;
	int verify_flag = 0;
	for(int i = 0; i < 7; i++)
	{
		if(capture.read(tmp_frame) == 0)
			printf("failed to read from camera \n");
	}
	tmp_frame.copyTo(tmp_frame2); 
	resize(tmp_frame2, tmp_frame3, Size(frame_width, frame_height), 0, 0, INTER_LINEAR);
	
	create_verify_feature_image(handle_detect,handle_verify, tmp_frame3, p_feature_new_2, frame_width, frame_height);
	for(int i = 0; i < person_number; i++)
	{
		cv_result_t cv_result = cv_verify_compare_feature(handle_verify, p_feature_new_1[i],
				p_feature_new_2, &score[i]);
		if (cv_result != CV_OK) 
			fprintf(stderr, "cv_verify_compare error : %d\n", cv_result);
		printf("score of %d is %f \n", i, score[i]);
	}
	max_func(score, person_number, max_score, max_id);
	fprintf(stderr, "max score: %f\n", max_score);
	if (max_score > DEFAULT_THRESHOLD)
	{
		fprintf(stderr, "right person, number is %d\n", max_id);
		verify_flag = 1;
		// send msg 1 to topic to start voice cmd node
	}
	else
	{
		fprintf(stderr, " you are not right person .\n");
		verify_flag = 0;
		//  send 0 to topic to stop voice cmd code
	}
	cv_verify_release_feature(p_feature_new_2);
	return verify_flag;
}

void max_func(const float *score, const int person_number, float &max_score, int &max_id)
{
	max_score = score[0];
	for(int i = 1; i < person_number; i++)
	{
		if(score[i] > max_score)
		{
			max_score = score[i];
			max_id = i;
		}
	}
}

int face_track(cv_handle_t &handle_track, VideoCapture &capture, const Point &expect, const int frame_width,  const int frame_height,
		 Publisher &servox_pub, Publisher &servoy_pub, Publisher &motor_pub)
{
	int track_flag = 0;
	cv_face_t *p_face = NULL;
	int face_count = 0;
	cv_result_t cv_result = CV_OK;
	Mat track_frame;
	while(1)
	{
		if( !capture.read(track_frame))
		{
			fprintf(stderr, "face track read failed\n");
			break;
		}
		Mat tmp_frame, color_frame;
		track_frame.copyTo(tmp_frame);
		resize(tmp_frame, tmp_frame, Size(frame_width, frame_height), 0, 0,
		       INTER_LINEAR);
		cvtColor(tmp_frame, color_frame, CV_BGR2BGRA);
		// realtime track
		cv_result = cv_face_track(handle_track, color_frame.data, CV_PIX_FMT_BGRA8888,
						color_frame.cols, color_frame.rows, color_frame.cols,
						CV_FACE_UP, &p_face, &face_count);
		if (cv_result != CV_OK) {
			fprintf(stderr, "cv_face_multi_track error : %d\n", cv_result);
			goto RETURN;
		}
		if(p_face)
		{
			track_flag = 1;
			int face_width = p_face[0].rect.right - p_face[0].rect.left;
			int face_height = p_face[0].rect.bottom - p_face[0].rect.top;
			int face_width_half = face_width >> 1;
			int face_height_half = face_height >> 1;
			Point center(p_face[0].rect.left + face_width_half, p_face[0].rect.top + face_height_half);
			Point target( -(center.x - expect.x), -(expect.y - center.y));

			rotate_cmd[0] = mkp_x(target.x);
			rotate_cmd[1] = mkp_y(target.y);
			motor_cmd = mk_motor(target.x, face_width);
			//motor_cmd = mk_motor2(target.x, target.y);
			printf(" face_width = %d target.x = %d target.y =%d motor_cmd = %c \n", face_width, target.x ,target.y, motor_cmd);
			//printf(" center.y = %d target.y = %d rotate_cmd[1] = %c\n", center.y ,target.y, rotate_cmd[1]);
			servox.data = rotate_cmd[0];
			servoy.data = rotate_cmd[1];
			motor.data = motor_cmd;
			//servox_pub.publish(servox);
			//servoy_pub.publish(servoy);
			//servoy_pub.publish(motor_cmd);
			cv_face_release_tracker_result(p_face, face_count);
		}
		else
		{
			track_flag = 0;
			fprintf(stderr, "track no face \n");
			// send msg to topic to stop voice cmd
			goto RETURN;
		}
	}

RETURN:
	// destroy track handle
	cv_face_destroy_tracker(handle_track);
}
	

/*************************************************************************
	> File Name: sendmultifile.cpp
	> Author: lengjing
	> Mail: jingleng77@163.com 
	> Created Time: Tue Nov 17 10:15:45 2015
 ************************************************************************/
/*
 * Add subcriber verify chatter from verify_tracking node. 2015.11.20
 * Add publisher voice cmd to verify_tracking node. 2015.11.20
 * Move client socket id to global varial to solve verfiy flag problem 2015.11.23 
 */
/* Use the newer ALSA API */
#define ALSA_PCM_NEW_HW_PARAMS_API
#include <alsa/asoundlib.h>
#include<stdio.h>
#include <stdlib.h>
#include <math.h>
#include<string.h>
#include <unistd.h>
#include<sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "ros/ros.h"
#include "std_msgs/Char.h"
#include <sys/time.h>

#define FRAME_LEN 640
using namespace ros;
const int channels = 1;
const int sample_rate = 16000;
const int bits_per_sample = 16;
const unsigned int speaktime = 60 * 1000000;
const int frames_per_period = 3200;
const int port = 9001;
char server_ip[20] = "10.0.1.77";

char motor_cmd = 'q';
std_msgs::Char motor; 
std_msgs::Char servox; 
std_msgs::Char servoy; 
std_msgs::Char track; 


volatile char verify_flag = '0';
volatile char track_flag = '0';
volatile int client_sockfd;

void convert(char speech_cmd, Publisher motor_pub, Publisher servox_pub, Publisher servoy_pub, Publisher track_pub)
{		
	switch(speech_cmd)
	{
		printf("recv speech_cmd %c\n", speech_cmd);
		fflush(stdout);
		case 'q':
			motor.data = speech_cmd;
			motor_pub.publish(motor);
			break;
		case 'a':
			motor.data = speech_cmd;
			motor_pub.publish(motor);
			break;
		case 'd': 
			motor.data = speech_cmd;
			motor_pub.publish(motor);
			break;
		case 'w': 
			motor.data = speech_cmd;
			motor_pub.publish(motor);
			break;
		case 's': 
			motor.data = speech_cmd;
			motor_pub.publish(motor);
			break;
		case 'h':
			servox.data = '+'; 
			servox_pub.publish(servox);
			break;
		case 'k':
			servox.data = '-';
			servox_pub.publish(servox);
			break;
		case 'u':
			servoy.data = '-';
			servoy_pub.publish(servoy);
			break;
		case 'j':
			servoy.data = '+';
			servoy_pub.publish(servoy);
			break;
		case 'f':
			track_flag = '1';
			track.data = track_flag;
			track_pub.publish(track);
			printf("publish track flag to verify_track node %c\n", track_flag);
			fflush(stdout);
			break;
		default :
			motor.data = 'q';
			servox.data = '0';
			servoy.data = '0';
			track_flag = '0';
			track.data = track_flag;
			track_pub.publish(track);
			//servox_pub.publish(servox);
			break;
	}
}

void *recv_cmd_func(void *args)
{
	NodeHandle n = *(NodeHandle *)args;
	Publisher motor_pub = n.advertise<std_msgs::Char>("motor_chatter", 1000);
	Publisher servox_pub = n.advertise<std_msgs::Char>("servox_chatter", 1000);
	Publisher servoy_pub = n.advertise<std_msgs::Char>("servoy_chatter", 1000);
	Publisher track_pub = n.advertise<std_msgs::Char>("track_chatter", 1);
	//printf("%d ", client_sockfd);
    //fflush(stdout);
	char speech_cmd;
	while(1)
	{
		int recvlen = 0;
		if((recvlen = recv(client_sockfd, &speech_cmd, 1, 0)) == 1)
		{
			printf("recv cmd is %c\n", speech_cmd);
			fflush(stdout);
			//motor.data = speech_cmd;
			//motor_pub.publish(motor);
			convert(speech_cmd, motor_pub, servox_pub, servoy_pub, track_pub);
#if 0
			printf("servox cmd is %c\n", servox.data);
			fflush(stdout);
			printf("servoy cmd is %c\n", servoy.data);
			fflush(stdout);
			printf("motor cmd is %c\n", motor.data);
			fflush(stdout);
#endif
			//speech_cmd = 'q';
		}
	}
}

void verifyCallback(const std_msgs::Char verify_msg)
{
	if(verify_msg.data == '0')
		verify_flag = '0';
	else
		verify_flag = '1';
}
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "cmd_control");
	ros::NodeHandle n;
	std::string serial_port;
	ros::param::param<std::string>("~port", serial_port, "/dev/ttyACM0");
	int baud;
	ros::param::param<int>("~baud", baud, 57600);
	int byte_per_sample = bits_per_sample >> 3;
	long loops;
	int rc;
	int size;
	snd_pcm_t *handle;
	snd_pcm_hw_params_t *params;
	unsigned int val;
	int dir;
	snd_pcm_uframes_t frames;
	char *buffer;
	char *speechbuf;

	/* Open PCM device for recording (capture). */
	rc = snd_pcm_open(&handle, "default",
			SND_PCM_STREAM_CAPTURE, 0);
	if (rc < 0) {
		fprintf(stderr,
				"unable to open pcm device: %s\n",
				snd_strerror(rc));
		exit(1);
	}

	/* Allocate a hardware parameters object. */
	snd_pcm_hw_params_alloca(&params);

	/* Fill it in with default values. */
	snd_pcm_hw_params_any(handle, params);

	/* Set the desired hardware parameters. */

	/* Interleaved mode */
	snd_pcm_hw_params_set_access(handle, params,
			SND_PCM_ACCESS_RW_INTERLEAVED);

	/* Signed 16-bit little-endian format */
	snd_pcm_hw_params_set_format(handle, params,
			SND_PCM_FORMAT_S16_LE);

	/* Two channels (stereo) */
	snd_pcm_hw_params_set_channels(handle, params, channels);

	/* 44100 bits/second sampling rate (CD quality) */
	//val = 44100;
	val = sample_rate;
	snd_pcm_hw_params_set_rate_near(handle, params,
			&val, &dir);

	/* Set period size to 32 frames. */
	frames = frames_per_period;
	snd_pcm_hw_params_set_period_size_near(handle,
			params, &frames, &dir);

	/* Write the parameters to the driver */
	rc = snd_pcm_hw_params(handle, params);
	if (rc < 0) {
		fprintf(stderr,
				"unable to set hw parameters: %s\n",
				snd_strerror(rc));
		exit(1);
	}

	/* Use a buffer large enough to hold one period */
	snd_pcm_hw_params_get_period_size(params,
			&frames, &dir);
	size = frames * channels * byte_per_sample; /* 2 bytes/sample, 1 channels */
	speechbuf = (char *) malloc(size);
	//printf("buffer size  %d\n", size);
 
	/* We want to loop for 5 seconds */
	snd_pcm_hw_params_get_period_time(params,
			&val, &dir);
	
	struct sockaddr_in remote_addr; //服务器端网络地址结构体  
	memset(&remote_addr,0,sizeof(remote_addr)); //数据初始化--清零  
	remote_addr.sin_family=AF_INET; //设置为IP通信  
	remote_addr.sin_addr.s_addr=inet_addr(server_ip);//服务器IP地址  
	remote_addr.sin_port=htons(port); //服务器端口号  

	/*创建客户端套接字--IPv4协议，面向连接通信，TCP协议*/  
	if((client_sockfd=socket(PF_INET,SOCK_STREAM,0))<0)  
	{  
		perror("socket failed\n");  
		return 1;  
	}  

	/*将套接字绑定到服务器的网络地址上*/  
	if(connect(client_sockfd,(struct sockaddr *)&remote_addr,sizeof(struct sockaddr))<0)  
	{  
		perror("connect failed\n");  
		return 1;  
	}  
	printf("connected to server\n");  
	printf("client socke id %d\n", client_sockfd);
	fflush(stdout);
	int len = 10 * FRAME_LEN;
	
	//printf("pcm size is %d", pcm_size);
	int totallen = 0;

	totallen = 0;
	int sendlen =0;
	int pcm_count = 0;
	int num = 0;

	pthread_t thread_recv;
	int err;
	void *tret;
	if((err = pthread_create(&thread_recv, NULL, &recv_cmd_func, &n)) != 0)
	{
		printf("thread recv erro : %s \n", strerror(err));
		return -1;
	}
	//sleep(1);
	
	//printf("thread creat is\n" );
    //fflush(stdout);
	ros::Subscriber sub = n.subscribe("verify_chatter", 1000, verifyCallback);
	while(1)
	{
		ros::spinOnce();
		//sleep(1);

		//printf("voice node get verify flag %c\n", verify_flag);
#if 0
		if(verify_flag == '0')
			continue;
#endif

		rc = snd_pcm_readi(handle, speechbuf, frames);
		if (rc == -EPIPE) {
			/* EPIPE means overrun */
			fprintf(stderr, "overrun occurred\n");
			snd_pcm_prepare(handle);
		} else if (rc < 0) {
			fprintf(stderr,
					"error from read: %s\n",
					snd_strerror(rc));
		} else if (rc != (int)frames) {
			fprintf(stderr, "short read, read %d frames\n", rc);
			break;
		}
		sendlen = send(client_sockfd, speechbuf, len, 0);
		if(sendlen < 0)
			printf("send failed \n");

	}
	if((err = pthread_join(thread_recv, &tret)) != 0)
	{
		printf("can not join thread_recv %s!\n", strerror(err));
		exit(-1);
	}
	//getchar();
	close(client_sockfd);
	snd_pcm_drain(handle);
	snd_pcm_close(handle);
	free(speechbuf);
	return 0;
}

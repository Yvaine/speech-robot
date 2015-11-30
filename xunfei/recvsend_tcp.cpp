/*
* ������д(iFly Auto Transform)�����ܹ�ʵʱ�ؽ�����ת���ɶ�Ӧ�����֡�
*/

#include <stdlib.h>
#include <stdio.h>
#include <windows.h>
#include <conio.h>
#include <errno.h>
#include <stdio.h>
//#include <WinSock2.h>

#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#ifdef _WIN64
#pragma comment(lib,"../libs/msc_x64.lib") //x64
#else
#pragma comment(lib,"../../libs/msc.lib") //x86
#endif
#pragma comment(lib, "ws2_32.lib")

#define	BUFFER_SIZE	4096
#define FRAME_LEN	640 
#define HINTS_SIZE  100
#define SPEECHSIZE 6400
#define RECOGSIZE 4096
#define NUM 5

int port = 9001;
/* �ϴ��û��ʱ� */
int upload_userwords()
{
	char*			userwords	=	NULL;
	unsigned int	len			=	0;
	unsigned int	read_len	=	0;
	FILE*			fp			=	NULL;
	int				ret			=	-1;

	fp = fopen("userwords1.txt", "rb");
	if (NULL == fp)										
	{
		printf("\nopen [userwords.txt] failed! \n");
		goto upload_exit;
	}

	fseek(fp, 0, SEEK_END);
	len = ftell(fp); //��ȡ��Ƶ�ļ���С
	fseek(fp, 0, SEEK_SET);  					
	
	userwords = (char*)malloc(len + 1);
	if (NULL == userwords)
	{
		printf("\nout of memory! \n");
		goto upload_exit;
	}

	read_len = fread((void*)userwords, 1, len, fp); //��ȡ�û��ʱ�����
	if (read_len != len)
	{
		printf("\nread [userwords.txt] failed!\n");
		goto upload_exit;
	}
	userwords[len] = '\0';
	
	MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret); //�ϴ��û��ʱ�
	if (MSP_SUCCESS != ret)
	{
		printf("\nMSPUploadData failed ! errorCode: %d \n", ret);
		goto upload_exit;
	}
	
upload_exit:
	if (NULL != fp)
	{
		fclose(fp);
		fp = NULL;
	}	
	if (NULL != userwords)
	{
		free(userwords);
		userwords = NULL;
	}
	
	return ret;
}

void run_iat( const char* session_begin_params)
{
	const char*		session_id					=	NULL;
	char			rec_result[BUFFER_SIZE]		=	{NULL};	
	char			hints[HINTS_SIZE]			=	{NULL}; //hintsΪ�������λỰ��ԭ�����������û��Զ���
	unsigned int	total_len					=	0; 
	int				aud_stat					=	MSP_AUDIO_SAMPLE_CONTINUE ;		//��Ƶ״̬
	int				ep_stat						=	MSP_EP_LOOKING_FOR_SPEECH;		//�˵���
	int				rec_stat					=	MSP_REC_STATUS_SUCCESS ;			//ʶ��״̬
	int				errcode						=	MSP_SUCCESS ;

	WSADATA wsaData;
	WORD sockVersion = MAKEWORD(2,2);
	if(WSAStartup(sockVersion, &wsaData) != 0)
		return;

	SOCKET serSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(serSocket == INVALID_SOCKET)
	{
		printf("socket failed \n");
		return ;
	}

	sockaddr_in serAddr;
	serAddr.sin_family = AF_INET;
	serAddr.sin_port = htons(port);
	serAddr.sin_addr.S_un.S_addr = INADDR_ANY;
	if(bind(serSocket, (LPSOCKADDR)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
	{
		printf("bind error!\n");
	    return ;
	}
	printf("����˿���\n");
	//��ʼ����
    if(listen(serSocket, 5) == SOCKET_ERROR)
    {
        printf("listen error !");
        return ;
    }
	
    SOCKET sClient;
	sockaddr_in remoteAddr;
	int nAddrLen = sizeof(remoteAddr);

	sClient = accept(serSocket, (SOCKADDR *)&remoteAddr, &nAddrLen);
    if(sClient == INVALID_SOCKET)
    {
        printf("accept error !");
    }
    printf("���ܵ�һ�����ӣ�%s \r\n", inet_ntoa(remoteAddr.sin_addr));

	char recvData[SPEECHSIZE];
	char sendData[RECOGSIZE];
	int recvlen, sendlen;

	printf("\n��ʼ������д ...\n");
	session_id = QISRSessionBegin(NULL, session_begin_params, &errcode); //��д����Ҫ�﷨����һ������ΪNULL
	if (MSP_SUCCESS != errcode)
	{
		printf("\nQISRSessionBegin failed! error code:%d\n", errcode);
		goto iat_exit;
	}
	unsigned int len = 10 * FRAME_LEN; // ÿ��д��200ms��Ƶ(16k��16bit)��1֡��Ƶ20ms��10֡=200ms��16k�����ʵ�16λ��Ƶ��һ֡�Ĵ�СΪ640Byte
	//ѭ����������
	while (1) 
	{
		total_len = 0;
		int block = 0;
		memset(rec_result, 0, strlen(rec_result));
		//int	aud_stat = MSP_AUDIO_SAMPLE_CONTINUE ;		//��Ƶ״̬
		int	ep_stat	= MSP_EP_LOOKING_FOR_SPEECH;		//�˵���
		int	rec_stat = MSP_REC_STATUS_SUCCESS ;			//ʶ��״̬
		int	errcode	= MSP_SUCCESS ;
		long crc = 0;
		int j = 0;
		while (1)
		{
			crc = 0;
			aud_stat = MSP_AUDIO_SAMPLE_CONTINUE;
			if (block == 0)
			{
				aud_stat = MSP_AUDIO_SAMPLE_FIRST;
			}
			block++;
			int ret = 0;
			int total_recv = 0;
			char *p_data = recvData;
			while (total_recv < SPEECHSIZE)
			{
				recvlen = recv(sClient, p_data, SPEECHSIZE - total_recv, 0);
				total_recv += recvlen;
				p_data += recvlen;
			}
			//printf("recv is %s", recvData);	
			//printf("recv len : %d", total_recv);
			for(int i = 0; i < SPEECHSIZE; i++)
			{
				crc += recvData[i];
			}
			printf("loop %d , crc is %ld \n", j, crc);
			j++;
#if 0
			if(recvlen < 0)
				printf("recevie from tk1 failed \n");
			printf(">>>>\n");
			ret = QISRAudioWrite(session_id, (const void *)&recvData, SPEECHSIZE, aud_stat, &ep_stat, &rec_stat);
			if (MSP_SUCCESS != ret)
			{
				printf("\nQISRAudioWrite failed! error code:%d\n", ret);
				goto iat_exit;
			}
			if (MSP_REC_STATUS_SUCCESS == rec_stat) //�Ѿ��в�����д���
			{
				printf("1.�Ѿ��в��ֽ����\n");
				const char *rslt = QISRGetResult(session_id, &rec_stat, 0, &errcode);
				if (MSP_SUCCESS != errcode)
				{
					printf("\nQISRGetResult failed! error code: %d\n", errcode);
					goto iat_exit;
				}
				if (NULL != rslt)
				{
					printf("1.��ȡ����ɹ�\n");
					unsigned int rslt_len = strlen(rslt);
					total_len += rslt_len;
					if (total_len >= BUFFER_SIZE)
					{
						printf("\nno enough buffer for rec_result !\n");
						goto iat_exit;
					}
					strncat(rec_result, rslt, rslt_len);
				}
			}
			if (MSP_EP_AFTER_SPEECH == ep_stat )
			{
				printf("end point of speech has been detected! \n");
				break;
			}
		}
		errcode = QISRAudioWrite(session_id, NULL, 0, MSP_AUDIO_SAMPLE_LAST, &ep_stat, &rec_stat);
		if (MSP_SUCCESS != errcode)
		{
			printf("\nQISRAudioWrite failed! error code:%d \n", errcode);
			goto iat_exit;	
		}

		while (MSP_REC_STATUS_COMPLETE != rec_stat) 
		{
			const char *rslt = QISRGetResult(session_id, &rec_stat, 0, &errcode);
			if (MSP_SUCCESS != errcode)
			{
				printf("\nQISRGetResult failed, error code: %d\n", errcode);
				goto iat_exit;
			}
			if (NULL != rslt)
			{
				unsigned int rslt_len = strlen(rslt);
				total_len += rslt_len;
				if (total_len >= BUFFER_SIZE)
				{
					printf("\nno enough buffer for rec_result !\n");
					goto iat_exit;
				}
				strncat(rec_result, rslt, rslt_len);
			}
			Sleep(150); //��ֹƵ��ռ��CPU
		}
		if(total_len != 0){
		printf("\n�˴�������д����\n");
		printf("=============================================================\n");
		printf("%s\n",rec_result);
		printf("=============================================================\n");
		}
		else
		{
			printf(" û�н��");
		}
#endif
		}
	}
iat_exit:

	QISRSessionEnd(session_id, hints);
}
		
int main(int argc, char* argv[])
{
	int			ret						=	MSP_SUCCESS;
	int			upload_on				=	1; //�Ƿ��ϴ��û��ʱ�
	const char* login_params			=	"appid = 5638844a, work_dir = ."; // ��¼������appid��msc���,��������Ķ�

	/*
	* sub:				����ҵ������
	* domain:			����
	* language:			����
	* accent:			����
	* sample_rate:		��Ƶ������
	* result_type:		ʶ������ʽ
	* result_encoding:	��������ʽ
	*
	* ��ϸ����˵������ġ�iFlytek MSC Reference Manual��
	*/
	const char* session_begin_params	=	"sub = iat, domain = iat, language = zh_ch, accent = mandarin, sample_rate = 16000, result_type = plain, result_encoding = gb2312, audio/L16; rate =1600";

	/* �û���¼ */
	ret = MSPLogin(NULL, NULL, login_params); //��һ���������û������ڶ������������룬����NULL���ɣ������������ǵ�¼����	
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed , Error code %d.\n",ret);
		goto exit; //��¼ʧ�ܣ��˳���¼
	}

	printf("\n########################################################################\n");
	printf("## ������д(iFly Auto Transform)�����ܹ�ʵʱ�ؽ�����ת���ɶ�Ӧ�����֡�##\n");
	printf("########################################################################\n\n");
	printf("��ʾʾ��ѡ��:�Ƿ��ϴ��û��ʱ�\n0:��ʹ��\n1:ʹ��\n");

	scanf("%d", &upload_on);
	if (upload_on)
	{
		printf("�ϴ��û��ʱ� ...\n");
		ret = upload_userwords();
		if (MSP_SUCCESS != ret)
			goto exit;	
		printf("�ϴ��û��ʱ�ɹ�\n");
	}
	run_iat(session_begin_params); //iflytek02��Ƶ����Ϊ���������ء�������ϴ����û��ʱ�ʶ����Ϊ���������ٿء���
exit:
	printf("��������˳� ...\n");
	_getch();
	MSPLogout(); //�˳���¼

	return 0;
}

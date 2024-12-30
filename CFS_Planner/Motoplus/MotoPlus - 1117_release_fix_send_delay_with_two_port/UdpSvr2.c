/* UdpSvr2.c */
/* Copyright 2009 YASKAWA ELECTRIC All Rights reserved. */

#include "motoPlus.h"
#include "ParameterExtraction.h"

// for API & FUNCTIONS
void moto_plus0_task(void);
void ap_UDP_Send_server(ULONG portNo);
void ap_UDP_Send_client_two(ULONG portNo);

#define PORT        11000
#define PORT1        11001 //port open for c++	
#define PORT2        11002 //port open for python
#define BUFF_MAX    1023

void moto_plus1_task(void)
{
	puts("Activate moto_plus1_task!");
	ap_UDP_Send_client_two(PORT1);
	mpSuspendSelf;
}

void moto_plus0_task(void)
{  
    puts("Activate moto_plus0_task!");
	ap_UDP_Send_server(PORT);

    mpSuspendSelf;
}

void ap_UDP_Send_client_two(ULONG portNo)
{
	int     sockHandle;
	struct  sockaddr_in     serverSockAddr;
	struct  sockaddr_in     clientSockAddr;
	int     sockHandle1;
	struct  sockaddr_in     serverSockAddr1;
	struct  sockaddr_in     clientSockAddr1;
	int     sizeofSockAddr;
	int     rc;
	float joint_pos_fbk[6] = { 0 };
	float joint_pos_cmd[6] = { 0 };
	int	first_time = 0;
	STATUS status;
	PULSE_TO_RAD PulseToRad;
	MP_PULSE_POS_RSP_DATA pulse_data_fbk;
	MP_PULSE_POS_RSP_DATA pulse_data_cmd;
	MP_PULSE_POS_RSP_DATA pulse_data_inc;
	MP_CTRL_GRP_SEND_DATA sData;
	MP_EXPOS_DATA moveData;
	char    buff_send[24];
	int     bytesSend;
	const char* SERVER_IP = "192.168.1.31";
	while (1)
	{
		// construct the UDP server
		sockHandle = mpSocket(AF_INET, SOCK_DGRAM, 0);
		if (sockHandle < 0)
			return;
		sockHandle1 = mpSocket(AF_INET, SOCK_DGRAM, 0);
		if (sockHandle1 < 0)
			return;

		memset(&serverSockAddr, 0, sizeof(serverSockAddr));
		serverSockAddr.sin_family = AF_INET;
		serverSockAddr.sin_addr.s_addr = mpInetAddr(SERVER_IP);
		serverSockAddr.sin_port = mpHtons(PORT1);

		memset(&serverSockAddr1, 0, sizeof(serverSockAddr1));
		serverSockAddr1.sin_family = AF_INET;
		serverSockAddr1.sin_addr.s_addr = mpInetAddr(SERVER_IP);
		serverSockAddr1.sin_port = mpHtons(PORT2);

		// get the pluse to rad value
		status = GP_getPulseToRad(0, &PulseToRad);
		sData.sCtrlGrp = 0;
		int i = 0;
		while (1)
		{
			// pubulish joint status
			mpGetPulsePos(&sData, &pulse_data_fbk);

			for (i = 0; i < 6; i++)
			{
				joint_pos_fbk[i] = pulse_data_fbk.lPos[i] / PulseToRad.PtoR[i];
			}
			memcpy(buff_send, joint_pos_fbk, sizeof(joint_pos_fbk));
			bytesSend = mpSendTo(sockHandle, buff_send, sizeof(joint_pos_fbk), 0,
				(struct sockaddr *)&serverSockAddr, sizeof(serverSockAddr));

			if (bytesSend != sizeof(joint_pos_fbk))
			{
				printf("send joint error %d", bytesSend);
				break;
			}

			bytesSend = mpSendTo(sockHandle1, buff_send, sizeof(joint_pos_fbk), 0,
				(struct sockaddr *)&serverSockAddr1, sizeof(serverSockAddr1));

			if (bytesSend != sizeof(joint_pos_fbk))
			{
				printf("send joint error %d", bytesSend);
				break;
			}
		}
		closeSockHandle:
		mpClose(sockHandle);
	}
}

void ap_UDP_Send_server(ULONG portNo)
{
    int     sockHandle;
    struct  sockaddr_in     serverSockAddr;
    struct  sockaddr_in     clientSockAddr;
    int     sizeofSockAddr;
    int     rc;
	float joint_pos_fbk[6] = { 0 };
	float joint_pos_cmd[6] = { 0 };
	int	first_time = 0;
	STATUS status;
	PULSE_TO_RAD PulseToRad;
	MP_PULSE_POS_RSP_DATA pulse_data_fbk;
	MP_PULSE_POS_RSP_DATA pulse_data_cmd;
	MP_PULSE_POS_RSP_DATA pulse_data_inc;
	MP_CTRL_GRP_SEND_DATA sData;
	MP_EXPOS_DATA moveData;
    while(1)
    {
		// construct the UDP server
        sockHandle = mpSocket(AF_INET, SOCK_DGRAM, 0);
        if (sockHandle < 0)
            return;

        memset(&serverSockAddr, 0, sizeof(serverSockAddr));
        serverSockAddr.sin_family = AF_INET;
        serverSockAddr.sin_addr.s_addr = INADDR_ANY;
        serverSockAddr.sin_port = mpHtons(portNo);

        rc = mpBind(sockHandle, (struct sockaddr *)&serverSockAddr, sizeof(serverSockAddr)); 
        if (rc < 0)
            goto closeSockHandle;
        
        memset(&clientSockAddr, 0, sizeof(clientSockAddr));
        sizeofSockAddr = sizeof(clientSockAddr);

		// get the pluse to rad value
		status = GP_getPulseToRad(0, &PulseToRad);
		sData.sCtrlGrp = 0;
		int i = 0;
        while (1)
        {
            int     bytesRecv;
            int     bytesSend;
            char    buff_recv[BUFF_MAX+1];
			char    buff_send[24];
			char    buff_cmd[8];
			char	joint_pos_cmd_byte[24];
			int		ret = 0;

            memset(buff_recv, 0, sizeof(buff_recv));
			memset(buff_send, 0, sizeof(buff_send));
			memset(buff_cmd, 0, sizeof(buff_cmd));
            bytesRecv = mpRecvFrom(sockHandle, buff_recv, BUFF_MAX, 0,
                              (struct sockaddr *)&clientSockAddr, &sizeofSockAddr);
			if (bytesRecv < 0)
			{
				break;
			}
			else
			{
				MP_START_JOB_SEND_DATA sStartJobData;
				MP_STD_RSP_DATA sRSPData;
				sStartJobData.sTaskNo = 1;
				strncpy(sStartJobData.cJobName, "INCREMENT_WHILE_WAIT", 21);
				sStartJobData.reserved[0] = 0;
				sStartJobData.reserved[1] = 0;
				sStartJobData.reserved[2] = 0;
				sStartJobData.reserved[3] = 0;
				sStartJobData.reserved[4] = 0;
				ret = mpStartJob(&sStartJobData, &sRSPData);
				if (ret != 0)
				{
					printf("mpStartJob error %d", ret);
				}

				memset(&moveData, 0x00, sizeof(moveData));
				memcpy(joint_pos_cmd, buff_recv, sizeof(joint_pos_cmd));
				memset(pulse_data_inc.lPos, 0, sizeof(pulse_data_inc.lPos));

				/* get pluse fbk*/
				mpGetPulsePos(&sData, &pulse_data_fbk);

				for (i = 0; i < 6; i++)
				{
					joint_pos_fbk[i] = pulse_data_fbk.lPos[i] / PulseToRad.PtoR[i];
				}
				//joint_pos_fbk[6] = joint_pos_cmd[6];
				//printf("joint des/fbk %f %f \n", joint_pos_cmd[5], joint_pos_fbk[5]);

				for (i = 0; i < 6; i++)
				{
					pulse_data_cmd.lPos[i] = (int)(joint_pos_cmd[i] * PulseToRad.PtoR[i]);
					pulse_data_inc.lPos[i] = pulse_data_cmd.lPos[i] - pulse_data_fbk.lPos[i];
				}

				moveData.m_ctrl_grp = 0; 
				moveData.s_ctrl_grp = 0;
				moveData.ctrl_grp = 1;
				moveData.grp_pos_info[0].pos_tag.data[0] = 0X3F;
				moveData.grp_pos_info[0].pos[0] = pulse_data_inc.lPos[0];
				moveData.grp_pos_info[0].pos[1] = pulse_data_inc.lPos[1];
				moveData.grp_pos_info[0].pos[2] = pulse_data_inc.lPos[2];
				moveData.grp_pos_info[0].pos[3] = pulse_data_inc.lPos[3];
				moveData.grp_pos_info[0].pos[4] = pulse_data_inc.lPos[4];
				moveData.grp_pos_info[0].pos[5] = pulse_data_inc.lPos[5];
				moveData.grp_pos_info[0].pos[6] = 0;
				moveData.grp_pos_info[0].pos[7] = 0;
				moveData.grp_pos_info[0].pos_tag.data[2] = 0;
				moveData.grp_pos_info[0].pos_tag.data[3] = MP_INC_PULSE_DTYPE;
				moveData.grp_pos_info[0].pos_tag.data[4] = 0;
				ret = mpExRcsIncrementMove(&moveData);
				if (ret != 0)
				{
					printf("send error %d", ret);
				}
			}
        }
		closeSockHandle:
        mpClose(sockHandle);
    }
}
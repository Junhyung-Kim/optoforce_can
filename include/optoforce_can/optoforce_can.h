#ifndef _OPTOFORCECAN_H
#define _OPTOFORCECAN_H

#include <ros/ros.h>
#include "stdio.h"
#include <cstdint>
#include <canlib.h>
#include <optoforce_can/FTsensor.h>

#define MAX_CHANNELS 63
#define FORCE_DIV	10.0
#define TORQUE_DIV	1000.0
#define canIOCTL_GET_EVENTHANDLE 14
#define ALARM_INTERVAL_IN_S     (1)
#define PRINTF_ERR(x)     printf x


typedef unsigned long DWORD;
typedef void* HANDLE;

typedef struct {
int        channel;
char       name[100];
DWORD      hwType;
canHandle  hnd;
int        hwIndex;
int        hwChannel;
int        isOnBus;
int        driverMode;
int        txAck;
} ChannelDataStruct;

typedef struct {
unsigned int       channelCount;
ChannelDataStruct  channel[MAX_CHANNELS];
} driverData;

driverData     m_channelData;
driverData    *m_DriverConfig = &m_channelData;


class optoforcecan_ROS
{
public:
	optoforcecan_ROS(ros::NodeHandle &nh) 
	{
		optoforce_pub_ = nh.advertise<optoforce_can::FTsensor>("/optoforce/ftsensor",0);
	}
	virtual ~optoforcecan_ROS(){}

	ros::Publisher optoforce_pub_;
	optoforce_can::FTsensor ft_msg_;

	///CAN Variable
	int   m_usedBaudRate = 1000000;
	canHandle      m_usedChannel = 0;
	unsigned int   m_usedId = 0x101;
	unsigned int   m_usedFlags = 0;
	unsigned int   m_Verbose = 0;

  HANDLE        th[MAX_CHANNELS + 1];
  DWORD         active_handle;
  char          c;
  canStatus     stat;
  bool canCheck;

	int Fx[3], Fy[3], Fz[3], Mx[3], My[3], Mz[3];
	unsigned int samplecounter[3];
	unsigned char change_hz=10;
	bool offset = true;
	int sensorid;
	int16_t Fdata[3][8];

	int framecounter[3];
	int cntforcount[3];

	////// Set Bus parameters/////
	void InitDriver(void)
	{
		int  i;
		canStatus  stat;

		// Initialize ChannelData.
		memset(m_channelData.channel, 0, sizeof(m_channelData.channel));
		for (i = 0; i < MAX_CHANNELS; i++) 
		{
			m_channelData.channel[i].isOnBus      = 0;
			m_channelData.channel[i].driverMode   = canDRIVER_NORMAL;
			m_channelData.channel[i].channel      = -1;
			m_channelData.channel[i].hnd          = canINVALID_HANDLE;
			m_channelData.channel[i].txAck        = 0; // Default is TxAck off
		}
		m_channelData.channelCount = 0;

		//
		// Enumerate all installed channels in the system and obtain their names
		// and hardware types.
		//

		//initialize CANlib
		canInitializeLibrary();

		//get number of present channels
		stat = canGetNumberOfChannels((int*)&m_channelData.channelCount);

		for (i = 0; (unsigned int)i < m_channelData.channelCount; i++) 
		{
			canHandle  hnd;
			//obtain some hardware info from CANlib
			m_channelData.channel[i].channel = i;
			canGetChannelData(i, canCHANNELDATA_CHANNEL_NAME,
							m_channelData.channel[i].name,
							sizeof(m_channelData.channel[i].name));
			canGetChannelData(i, canCHANNELDATA_CARD_TYPE,
							&m_channelData.channel[i].hwType,
							sizeof(DWORD));

			//open CAN channel
			hnd = canOpenChannel(i, canOPEN_ACCEPT_VIRTUAL);
			if (hnd < 0) 
			{
				// error
				PRINTF_ERR(("ERROR canOpenChannel() in initDriver() FAILED Err= %d. <line: %d>\n",
							hnd, __LINE__));
			}
			else 
			{
				m_channelData.channel[i].hnd = hnd;
				if ((stat = canIoCtl(hnd, canIOCTL_FLUSH_TX_BUFFER, NULL, NULL)) != canOK)
					PRINTF_ERR(("ERROR canIoCtl(canIOCTL_FLUSH_TX_BUFFER) FAILED, Err= %d <line: %d>\n",
								stat, __LINE__));
			}

			//set up the bus
			if (i == 0) {
			switch(m_usedBaudRate) 
			{
				case 1000000:
				m_usedBaudRate = canBITRATE_1M;
				break;
				case 500000:
				m_usedBaudRate = canBITRATE_500K;
				break;
				case 250000:
				m_usedBaudRate = canBITRATE_250K;
				break;
				case 125000:
				m_usedBaudRate = canBITRATE_125K;
				break;
				case 100000:
				m_usedBaudRate = canBITRATE_100K;
				break;
				case 62500:
				m_usedBaudRate = canBITRATE_62K;
				break;
				case 50000:
				m_usedBaudRate = canBITRATE_50K;
				break;
				default:
				printf("Baudrate set to 125 kbit/s. \n");
				m_usedBaudRate = canBITRATE_125K;
				break;
			}
			}
			
			//set the channels busparameters
			stat = canSetBusParams(hnd, m_usedBaudRate, 0, 0, 0, 0, 0);
			if (stat < 0) 
			{
				PRINTF_ERR(("ERROR canSetBusParams() in InitDriver(). Err = %d <line: %d>\n",
							stat, __LINE__));
			}

			for (i = 1; i < (m_channelData.channelCount + 1); i++) 
			{
				HANDLE tmp;
				//go on bus (every channel)
				stat = canBusOn(m_channelData.channel[i-1].hnd);
				if (stat < 0) 
				{
					PRINTF_ERR(("ERROR canBusOn(). Err = %d <line: %d>\n", stat, __LINE__));
				}
				else 
				{
					m_DriverConfig->channel[i-1].isOnBus = 1;
				}
			}
		}
		printf("\n");
	}
	void printDriverConfig( void )
	{
		unsigned int i;

		printf("\nDriver Configuration:\n  ChannelCount=%u\n", m_DriverConfig->channelCount);
		for (i = 0; i < m_DriverConfig->channelCount; i++) 
		{

			printf("  %s : Channel %d, isOnBus=%d, Baudrate=%u",
				m_DriverConfig->channel[i].name,
				m_DriverConfig->channel[i].channel,
				m_DriverConfig->channel[i].isOnBus,
				m_usedBaudRate);

			switch(m_usedBaudRate) 
			{
				case canBITRATE_1M:
					printf("canBITRATE_1M");
					break;
				case canBITRATE_500K:
					printf("canBITRATE_500K");
					break;
				case canBITRATE_250K:
					printf("canBITRATE_250K");
					break;
				case canBITRATE_125K:
					printf("canBITRATE_125K");
					break;
				case canBITRATE_100K:
					printf("canBITRATE_100K");
					break;
				case canBITRATE_62K:
					printf("canBITRATE_62K");
					break;
				case canBITRATE_50K:
					printf("canBITRATE_50K");
					break;
				default:
					printf("UNKNOWN");
			}
			printf("\n    ");

			if (m_DriverConfig->channel[i].driverMode == canDRIVER_NORMAL) 
			{
				printf("Drivermode=canDRIVER_NORMAL\n");
			} 
			else 
			{
				printf ("Drivermode=canDRIVER_SILENT\n");
			}
		}
		printf("\n\n");
	}

	void calcForceMoment()
	{
		
		alarm(ALARM_INTERVAL_IN_S);
  
		unsigned int    i;
		unsigned int    j;
		long            id;
		unsigned char   data[8];
		unsigned int    dlc;
		unsigned int    flags;
		DWORD           time;
		int             moreDataExist;
		
		sensorid = 0;
		
    do {
          moreDataExist = 0;
          for (i = 0; i < m_channelData.channelCount; i++) 
          {
            stat = canRead(m_channelData.channel[i].hnd, &id, &data[0], &dlc, &flags, &time);
            if(canCheck == false)
            {
                if(stat == canOK)
                {
                  printf("channel %i OK,  %i\n", m_channelData.channel[i].hnd, stat);
                }
                else
                {
                  printf("channel %i Err", m_channelData.channel[i].hnd, stat);  
                }   
            }
          }

          canCheck = true;	

          switch (stat) 
          {
            case canOK:

            if ((flags & canMSG_RTR) == 0) 
            { 
              if (id==0x111) 
              {
                sensorid = 1;
              }
              else if (id == 0x100) 
              {
                sensorid = 0;
              }
              else 
              {
                sensorid = 2;
              }
              if (data[0] == 170 && data[1] == 7) 
              {
                samplecounter[sensorid] = (((unsigned int)data[4]) << 8) + ((unsigned int)data[5]);
                framecounter[sensorid] = 1;
              }
              else if (framecounter[sensorid] == 1) 
              {
                Fdata[sensorid][0] = (((unsigned int)data[0])<<8)+ ((unsigned int)data[1]);
                Fdata[sensorid][1] = (((unsigned int)data[2]) << 8) + ((unsigned int)data[3]);
                Fdata[sensorid][2] = (((unsigned int)data[4]) << 8) + ((unsigned int)data[5]);
                Fdata[sensorid][3] = (((unsigned int)data[6]) << 8) + ((unsigned int)data[7]);
                Fdata[sensorid][4] = (((unsigned int)data[0]) << 8) + ((unsigned int)data[1]);
                Fdata[sensorid][5] = (((unsigned int)data[2]) << 8) + ((unsigned int)data[3]);
                framecounter[sensorid] = 0;
              
               /* cntforcount[sensorid]++;
                if(cntforcount[sensorid]%5 == 0) 
                {
                  printf("id : %x sample: %04x time : %04f  Fx: %.1f  Fy: %.1f  Fz: %.1f  Mx: %.2f  My: %.2f  Mz: %.2f \n", id, samplecounter[sensorid], (float)time / 1000, (float)Fdata[sensorid][0] / 10.0, (float)Fdata[sensorid][1] / 10.0, (float)Fdata[sensorid][2] / 10.0, (float)Fdata[sensorid][3] / 1000.0, (float)Fdata[sensorid][4] / 1000.0, (float)Fdata[sensorid][5] / 1000.0);
                  cntforcount[sensorid] = 0;
                }*/
              }
            }
            moreDataExist = 1;
            break;

            case canERR_NOMSG:
            // No more data on this handle
            break;

            default:
            PRINTF_ERR(("ERROR canRead() FAILED, Err= %d <line: %d>\n", stat, __LINE__));
            break;
          }
        } while (moreDataExist);
	}
  
  void FTpublish()
  {
    ft_msg_.Fx_R = Fdata[0][0]/FORCE_DIV;
    ft_msg_.Fy_R = Fdata[0][1]/FORCE_DIV;
    ft_msg_.Fz_R = Fdata[0][2]/FORCE_DIV;
    ft_msg_.Tx_R = Fdata[0][3]/TORQUE_DIV;
    ft_msg_.Ty_R = Fdata[0][4]/TORQUE_DIV;
    ft_msg_.Tz_R = Fdata[0][5]/TORQUE_DIV;

    ft_msg_.Fx_L = Fdata[1][0]/FORCE_DIV;
    ft_msg_.Fy_L = Fdata[1][1]/FORCE_DIV;
    ft_msg_.Fz_L = Fdata[1][2]/FORCE_DIV;
    ft_msg_.Tx_L = Fdata[1][3]/TORQUE_DIV;
    ft_msg_.Ty_L = Fdata[1][4]/TORQUE_DIV;
    ft_msg_.Tz_L = Fdata[1][5]/TORQUE_DIV;

    optoforce_pub_.publish(ft_msg_);
  }
};

#endif

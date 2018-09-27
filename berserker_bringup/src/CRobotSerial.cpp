#include "RobotStruct.h"
#include "CRobotSerial.h"

CRobotSerial::CRobotSerial()
	: m_ID(0x01), m_cFrameHead_1(0xF5), m_cFrameHead_2(0xAA)
{
	m_ResetRcvData();
	m_ResetSenData();
	m_ResetRcvBuf();

	m_cLast = 0;
	m_cLastLast = 0;
	m_cLastLastLast = 0;
	m_nFrameLength = 0;
	m_nRcvIndex = 0;
	m_pSenBufL = 0;

	ParsedFrameNum = 0;
}

CRobotSerial::CRobotSerial(byte ID)
	: m_ID(ID), m_cFrameHead_1(0xF5), m_cFrameHead_2(0xAA)
{
	m_ResetRcvData();
	m_ResetSenData();
	m_ResetRcvBuf();

	m_cLast = 0;
	m_cLastLast = 0;
	m_cLastLastLast = 0;
	m_nFrameLength = 0;
	m_nRcvIndex = 0;
	m_pSenBufL = 0;

	ParsedFrameNum = 0;
}

CRobotSerial::CRobotSerial(byte ID, byte FrameHead_1, byte FrameHead_2)
	: m_ID(ID), m_cFrameHead_1(FrameHead_1), m_cFrameHead_2(FrameHead_2)
{
	m_ResetRcvData();
	m_ResetSenData();
	m_ResetRcvBuf();

	m_cLast = 0;
	m_cLastLast = 0;
	m_cLastLastLast = 0;
	m_nFrameLength = 0;
	m_nRcvIndex = 0;
	m_pSenBufL = 0;

	ParsedFrameNum = 0;
}

CRobotSerial::~CRobotSerial()
{
}

bool CRobotSerial::m_ReadFrame(const byte buf)
{
	if (m_nRcvIndex < 4)
	{
		//���Ұ�ͷ 0xF5 0xAA , ����ID
		if (buf > 0 && m_cLast == m_ID && m_cLastLast == m_cFrameHead_2 && m_cLastLastLast == m_cFrameHead_1)
		{
			m_pRcvBuf[0] = m_cFrameHead_1;
			m_pRcvBuf[1] = m_cFrameHead_2;
			m_pRcvBuf[2] = m_ID;
			m_pRcvBuf[3] = buf;
			m_nFrameLength = buf;
			m_nRcvIndex = 4;
			m_cLast = 0x00;
			m_cLastLast = 0x00;
			m_cLastLastLast = 0x00;
			if (m_nFrameLength > MAX_RcvBUFF_L) //�����������ȴ��ڻ��������� ����
			{
				m_nRcvIndex = 0;
			}
			return false;
		}
		m_cLastLastLast = m_cLastLast;
		m_cLastLast = m_cLast;
		m_cLast = buf;
	}
	else
	{
		//����һ��֡, ���� ParseFrame ������
		if (m_nRcvIndex > m_nFrameLength - 2) //m_nRcvIndex==����У��λ�� �Ѿ�������� ʱ �ͽ������ParseFrame
		{
			m_pRcvBuf[m_nRcvIndex] = buf; //У��λ
			m_nRcvIndex = 0;
			if (m_DelDataZero(m_pRcvBuf, m_nFrameLength)) //������ȥͷ��ȥ�㣬ȥУ�飬��������RcvData��
			{
				//m_ParseFrame(RcvData, RcvData_L);//����RcvData�����ݽ���
				return true;
			}
			else
			{
				return false;
			}
			return false;
		}
		else
		{
			//�����յ������ݷ��뻺����
			m_pRcvBuf[m_nRcvIndex] = buf;
			m_nRcvIndex++;
			return false;
		}
	}
	return false;
}

void CRobotSerial::m_ParseFrame(byte *buf, int length)
{
	// SubMsg.MsgType = RcvData[0];
	// SubMsg.RobotState = RcvData[1];

	// SubMsg.Left_motor.motor_current = bytesToInt(RcvData + 2);
	// SubMsg.Left_motor.motor_encoder = bytesToInt(RcvData + 6);
	// SubMsg.Right_motor.motor_current = bytesToInt(RcvData + 10);
	// SubMsg.Right_motor.motor_encoder = bytesToInt(RcvData + 14);
	// SubMsg.Sonar.sonar_1 = bytesToFloat(RcvData + 18);
	// SubMsg.Sonar.sonar_2 = bytesToFloat(RcvData + 22);
	// SubMsg.Sonar.sonar_3 = bytesToFloat(RcvData + 26);
	// SubMsg.Sonar.sonar_4 = bytesToFloat(RcvData + 30);
	// SubMsg.IMU.patch = bytesToFloat(RcvData + 34);
	// SubMsg.IMU.roll = bytesToFloat(RcvData + 38);
	// SubMsg.IMU.yaw = bytesToFloat(RcvData + 42);
	// SubMsg.IO.input_1 = RcvData[46];
	// SubMsg.IO.input_2 = RcvData[47];
	// SubMsg.IO.output_1 = RcvData[48];
	// SubMsg.IO.output_2 = RcvData[49];
}

int CRobotSerial::m_BuildFrame(byte *buf, int length)
{
	byte b0 = 0;
	byte b1 = 0;

	SenData[0] = m_cFrameHead_1;
	SenData[1] = m_cFrameHead_2;
	SenData[2] = m_ID;

	int j = 4;
	for (int i = 0; i < length; i++)
	{
		b0 = buf[i];
		if (b0 == m_cFrameHead_2 && b1 == m_cFrameHead_1)
		{
			SenData[j] = b0;
			j++;
			SenData[j] = 0x00;
			j++;
		}
		else
		{
			SenData[j] = b0;
			j++;
		}
		b1 = b0;
	}
	SenData[3] = j + 1;

	SenData[j] = m_AddDataSum(SenData, j);

	SenData_L = j + 1;
	return SenData_L;
}

bool CRobotSerial::m_DelDataZero(byte *data, const int data_L)
{
	if (!m_CheckDataSum(data, data_L, data[data_L - 1]))
	{
		return false; //У�鲻ͨ�� ������ɹ�
	}
	byte b0 = 0x00;
	byte b1 = 0x00;
	byte b2 = 0x00;

	int j = 0;
	for (int i = 4; i < data_L - 1; i++)
	{
		b0 = data[i];
		if (b0 == 0x00 && b1 == m_cFrameHead_2 && b2 == m_cFrameHead_1)
		{
			; //����
		}
		else
		{
			RcvData[j] = b0;
			j++;
		}
		b2 = b1;
		b1 = b0;
	}
	RcvData_L = j;
	ParsedFrameNum++; //�ۻ��ɹ���������

	return true; //У��ͨ�� ����ɹ�
}

void CRobotSerial::m_ResetRcvBuf()
{
	for (int i = 0; i < MAX_SenBUFF_L; i++)
	{
		m_pRcvBuf[i] = 0x00;
	}
	m_nRcvIndex = 0;

	m_cLast = 0x00;
	m_nFrameLength = 0x00;
}

void CRobotSerial::m_ResetRcvData()
{
	for (int i = 0; i < MAX_RcvBUFF_L; i++)
	{
		RcvData[i] = 0x00;
	}
	RcvData_L = 0;
}

void CRobotSerial::m_ResetSenData()
{
	for (int i = 0; i < MAX_SenBUFF_L; i++)
	{
		SenData[i] = 0x00;
	}
	SenData_L = 0;
}

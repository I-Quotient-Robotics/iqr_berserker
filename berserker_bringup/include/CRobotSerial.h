#pragma once
#include "CSerial.h"
#include "RobotStruct.h"
#include "function.h"

#define MAX_RcvBUFF_L 64
#define MAX_SenBUFF_L 64 

class CRobotSerial :
	public CSerial
{
public:
	CRobotSerial();
	CRobotSerial(byte ID);
	CRobotSerial(byte ID, byte FrameHead_1, byte FrameHead_2);

	~CRobotSerial();

	const byte m_ID;

	byte RcvData[MAX_RcvBUFF_L];
	unsigned int RcvData_L;
	byte SenData[MAX_SenBUFF_L];
	unsigned int SenData_L;

	bool m_ReadFrame(const byte buf);
	void m_ParseFrame(byte* buf, int length);
	int  m_BuildFrame(byte* buf, int length);

	RobotSubMsg SubMsg;

	unsigned long int  ParsedFrameNum;

protected:
	byte m_cLast; 
	byte m_cLastLast; 
	byte m_cLastLastLast; 
	byte m_nFrameLength; 

	byte m_pRcvBuf[MAX_RcvBUFF_L];
	unsigned int m_nRcvIndex;
	byte m_pSenBuf[MAX_SenBUFF_L];
	unsigned int m_pSenBufL;

	bool m_DelDataZero(byte *data, const int data_L);

	void m_ResetRcvBuf();
	void m_ResetRcvData();
	void m_ResetSenData();


private:
	const byte m_cFrameHead_1;
	const byte m_cFrameHead_2;
};


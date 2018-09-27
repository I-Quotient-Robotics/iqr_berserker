#include "RobotStruct.h"
#include "CSerial.h"


CSerial::CSerial()
{
}

CSerial::~CSerial()
{
}



bool CSerial::m_CheckDataSum(byte * data, const int data_L, const byte checksum)
{
	byte sum = 0x00;
	for (int i = 0; i < data_L - 1; i++)
	{
		sum += data[i];
	}
	if (sum == checksum)
		return true;
	else
		return false;
}

byte CSerial::m_AddDataSum(byte * data, const int data_L)
{
	byte sum = 0x00;
	for (int i = 0; i < data_L; i++)
	{
		sum += data[i];
	}
	return sum;
}



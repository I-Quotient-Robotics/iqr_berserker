// 
// 
// 

#include "function.h"

//intתbyte
void  intToByte(int i, byte *bytes)
{
	bytes[3] = (byte)(0xff & i);
	bytes[2] = (byte)((0xff00 & i) >> 8);
	bytes[1] = (byte)((0xff0000 & i) >> 16);
	bytes[0] = (byte)((0xff000000 & i) >> 24);
	return;
}

//byteתint
int bytesToInt(byte* bytes)
{
	int addr = bytes[3];
	addr |= (bytes[2] << 8);
	addr |= (bytes[1] << 16);
	addr |= (bytes[0] << 24);
	return addr;
}

//floatתbyte
void floatToByte(float f, byte *bytes)
{
	//��float���͵�ָ��ǿ��ת��Ϊunsigned char��
	byte *pdata = (byte *)&f;
	for (int i = 3; i >= 0; i--)
	{
		bytes[i] = *pdata++;//����Ӧ��ַ�е����ݱ��浽unsigned char������       
	}
}

//byteתfloat
float bytesToFloat(byte *b)
{
	int accum = 0;
	accum = accum | (b[3] & 0xff) << 0;
	accum = accum | (b[2] & 0xff) << 8;
	accum = accum | (b[1] & 0xff) << 16;
	accum = accum | (b[0] & 0xff) << 24;

	return intBitsToFloat(accum);
}

float intBitsToFloat(int i)
{
	union
	{
		int i;
		float f;
	} u;
	u.i = i;
	return u.f;
}

#ifndef _CSerial_h
#define _CSerial_h

typedef unsigned char byte;

class CSerial
{
public:
	CSerial();
	virtual ~CSerial();

	virtual bool m_ReadFrame(const byte buf) = 0;
	virtual void m_ParseFrame(byte* buf, int length) = 0;
	virtual int  m_BuildFrame(byte* buf, int length) = 0;


protected:
	bool m_CheckDataSum(byte *data, const int data_L, const byte checksum);
	byte m_AddDataSum(byte *data, const int data_L);
		
private:

};
#endif // !CSerial





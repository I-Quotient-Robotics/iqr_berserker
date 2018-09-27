// function.h

#ifndef _FUNCTION_h
#define _FUNCTION_h

typedef unsigned char byte;
//intתbyte
void  intToByte(int i, byte *bytes);

//byteתint
int bytesToInt(byte* bytes);

//floatתbyte
void floatToByte(float f, byte *bytes);

//byteתfloat
float bytesToFloat(byte *b);

float intBitsToFloat(int i);

#endif


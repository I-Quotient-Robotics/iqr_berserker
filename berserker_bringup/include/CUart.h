#ifndef _CUART_H_
#define _CUART_H_
#include <stdio.h>  /*标准输入输出定义*/
#include <stdlib.h> /*标准函数库定义*/
#include <unistd.h> /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <errno.h>   /*错误号定义*/
#include <string.h>

#include <iostream>
#include <thread>
#include <mutex>

typedef unsigned char byte;

class CUart
{
  public:
    CUart(char *port);
    CUart(char *port, int speed);
    ~CUart();

    bool m_set(int speed, int flow_ctrl, int databits, int stopbits, int parity);

    int m_read(byte *buf, int len);

    int m_send(byte *buf, int len);

    void m_close();

  protected:
    //bool m_Init(int speed);
    bool m_open(char *port);
    bool m_defaultSet(int speed);

  private:
    int m_fd;
};

#endif
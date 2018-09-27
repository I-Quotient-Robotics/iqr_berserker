#include "CUart.h"

CUart::CUart(char *port)
{
    if(!m_open(port))
    exit(-1);
}

CUart::CUart(char *port, int speed)
{
    if(!m_open(port))
    exit(-1);
    if(!m_defaultSet(speed))
    exit(-1);
}

CUart::~CUart()
{
    m_close();
}

bool CUart::m_open(char *port)
{
    m_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (m_fd == -1)
    {
        perror("Can't Open Serial Port");
        return false;
    }
    //TODO: 恢复串口为阻塞状态 非阻塞FNDELAY 
    if (fcntl(m_fd,F_SETFL,0) < 0)
    {
        printf("fcntl failed!\n");
        return false;
    }

    //测试是否为终端设备
    if (isatty(STDIN_FILENO) == 0)
    {
        return false;
    }

    printf("fd->open=%d\n", m_fd);
    return true;
}

bool CUart::m_defaultSet(int speed)
{
    //设置串口数据帧格式
    if (m_set(speed, 0, 8, 1, 'N') == false)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void CUart::m_close()
{
    close(m_fd);
}

bool CUart::m_set(int speed, int flow_ctrl, int databits, int stopbits, int parity)
{
    int status;
    int speed_arr[] = {B115200, B19200, B9600, B4800, B2400, B1200};
    int name_arr[] = {115200, 19200, 9600, 4800, 2400, 1200};

    struct termios options;

    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  
    */
    if (tcgetattr(m_fd, &options) != 0)
    {
        perror("SetupSerial 1");
        return false;
    }

    //设置串口输入波特率和输出波特率
    for (unsigned int i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch (flow_ctrl)
    {

    case 0: //不使用流控制
        options.c_cflag &= ~CRTSCTS;
        break;
    case 1: //使用硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2: //使用软件流控制
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "Unsupported data size\n");
        return false;
    }
    //设置校验位
    switch (parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O': //设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E': //设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported parity\n");
        return false;
    }
    // 设置停止位
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported stop bits\n");
        return false;
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1;  /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(m_fd, TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(m_fd, TCSANOW, &options) != 0)
    {
        perror("com set error!\n");
        return false;
    }
    return true;
}

int CUart::m_read(byte *buf, int len)
{
    //return read(m_fd, buf, len);
    int len_,fs_sel;    
    fd_set fs_read;    
       
    struct timeval time;    
       
    FD_ZERO(&fs_read);    
    FD_SET(m_fd,&fs_read);    
       
    time.tv_sec = 0;    
    time.tv_usec = 100000;    
       
    //使用select实现串口的多路通信    
    fs_sel = select(m_fd+1,&fs_read,NULL,NULL,&time);    
    //printf("fs_sel = %d\n",fs_sel);    
    if(fs_sel)    
    {    
        len_ = read(m_fd,buf,len);    
        //printf("I am right!(version1.2) len = %d fs_sel = %d\n",len,fs_sel);    
        return len_;    
    }    
    else    
    {    
        return -1;    
    } 
}

int CUart::m_send(byte *buf, int len)
{
    int _len = write(m_fd, buf, len);
    if (_len == len)
    {
        return _len;
    }
    else
    {
        tcflush(m_fd, TCOFLUSH);
        return -1;
    }
}

#include "CRobot.h"

CRobot::CRobot(char *port) : m_iSendRate(50)
{
    m_pUart = new CUart(port, 115200);
    m_pSerial = new CRobotSerial(0x01);

    m_flastLSpeed = 0.0;
    m_flastRspeed = 0.0;

    m_bSonar_ = true;

    m_bExit_ = true;
    m_bExit_2 = true;
}
CRobot::~CRobot()
{
    m_pSerial->~CRobotSerial();
    m_pUart->~CUart();
}

void CRobot::run()
{
    int _len = 0;

    byte buf[32] = {
        0,
    };
    while (m_bExit_)
    {

        _len = m_pUart->m_read(buf, 1);

        for (int i = 0; i < _len; i++)
        {
            if (m_pSerial->m_ReadFrame(buf[i]))
            {
                mtx_1.lock();

                SubMsg.MsgType = m_pSerial->RcvData[0];
                SubMsg.RobotState = m_pSerial->RcvData[1];

                SubMsg.Odom.x = bytesToFloat(m_pSerial->RcvData + 2);
                SubMsg.Odom.y = bytesToFloat(m_pSerial->RcvData + 6);
                SubMsg.Odom.th = bytesToFloat(m_pSerial->RcvData + 10);
                SubMsg.Odom.v = bytesToFloat(m_pSerial->RcvData + 14);
                SubMsg.Odom.vth = bytesToFloat(m_pSerial->RcvData + 18);

                // SubMsg.Sonar.sonar_1 = bytesToFloat(m_pSerial->RcvData + 18);
                // SubMsg.Sonar.sonar_2 = bytesToFloat(m_pSerial->RcvData + 22);
                // SubMsg.Sonar.sonar_3 = bytesToFloat(m_pSerial->RcvData + 26);
                // SubMsg.Sonar.sonar_4 = bytesToFloat(m_pSerial->RcvData + 30);
                // SubMsg.IMU.patch = bytesToFloat(m_pSerial->RcvData + 34);
                // SubMsg.IMU.roll = bytesToFloat(m_pSerial->RcvData + 38);
                // SubMsg.IMU.yaw = bytesToFloat(m_pSerial->RcvData + 42);

                mtx_1.unlock();
            }
        }
        usleep(10);
    }
}
void CRobot::setRobotSpeed(const float v, const float r)
{
    mtx_2.lock();
    m_flastLSpeed = v;
    m_flastRspeed = r;
    mtx_2.unlock();
}

bool CRobot::setRobotState(const RobotState state)
{
    if (mtx_2.try_lock())
    {
        m_bRobotState = byte(state);
        mtx_2.unlock();
        return true;
    }
    else
    {
        return false;
    }
}
bool CRobot::setMsgType(const byte type)
{
    if (mtx_2.try_lock())
    {
        m_bMsgType = byte(type);
        mtx_2.unlock();
        return true;
    }
    else
    {
        return false;
    }
}

float CRobot::readIMU(const IMU imu)
{
    float re = 0.0;
    switch (imu)
    {
    case PITCH:
        mtx_1.lock();
        re = SubMsg.IMU.patch;
        mtx_1.unlock();
        break;
    case ROLL:
        mtx_1.lock();
        re = SubMsg.IMU.roll;
        mtx_1.unlock();
        break;
    case YAW:
        mtx_1.lock();
        re = SubMsg.IMU.yaw;
        mtx_1.unlock();
        break;
    default:
        break;
    }
    return re;
}
float CRobot::readOdom(const Odom odom)
{
    float re = 0.0;
    switch (odom)
    {
    case X:
        mtx_1.lock();
        re = SubMsg.Odom.x;
        mtx_1.unlock();
        break;
    case Y:
        mtx_1.lock();
        re = SubMsg.Odom.y;
        mtx_1.unlock();
        break;
    case TH:
        mtx_1.lock();
        re = SubMsg.Odom.th;
        mtx_1.unlock();
        break;
    case V:
        mtx_1.lock();
        re = SubMsg.Odom.v;
        mtx_1.unlock();
        break;
    case VTH:
        mtx_1.lock();
        re = SubMsg.Odom.vth;
        mtx_1.unlock();
        break;
    default:
        break;
    }
    return re;
}

void CRobot::run_2()
{
    static clock_t now_time = 0;
    static clock_t last_time = 0;
    static byte PubMsg[10] = {
        0,
    };
    while (m_bExit_2)
    {
        //now_time = clock();
        //if ((now_time - last_time) > (CLOCKS_PER_SEC / m_iSendRate))
        //{
        mtx_2.lock();
        PubMsg[0] = m_bMsgType;
        PubMsg[1] = m_bRobotState;
        floatToByte(m_flastLSpeed, PubMsg + 2);
        floatToByte(m_flastRspeed, PubMsg + 6);
        mtx_2.unlock();
        m_pSerial->m_BuildFrame(PubMsg, 10);
        m_pUart->m_send(m_pSerial->SenData, m_pSerial->SenData_L);
        last_time = now_time;
        // }
        usleep(1000 * (1000 / (m_iSendRate)));
    }
}
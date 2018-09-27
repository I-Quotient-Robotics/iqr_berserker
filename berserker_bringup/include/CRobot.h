#pragma once
#include <iostream>
#include <thread>
#include <mutex>
#include <sys/timeb.h>
#include <time.h>
#include "CRobotSerial.h"
#include "CUart.h"
#include "RobotStruct.h"
#include "function.h"
#include "thread.h"


enum RobotState {STOP, RUN, TURN_LEFT, TURN_RIGHT, ERRO};
enum IMU {PITCH, ROLL, YAW};
enum Odom {X, Y, TH, V, VTH};


using namespace std;

class CRobot : public Thread
{
  public:
    CRobot(char *port);
    ~CRobot();

    void setRobotSpeed(const float v, const float r);
    bool setRobotState(const RobotState state);
    bool setMsgType(const byte type);
   
    float readIMU(const IMU imu);
    float readOdom(const Odom odom);

    void end();

  protected:
    void run();
    void run_2();
    bool checkSonar();
    

    volatile float m_flastLSpeed;
    volatile float m_flastRspeed;
    volatile byte m_bMsgType;
    volatile byte m_bRobotState;
    volatile RobotSubMsg SubMsg;

  private:
    const int m_iSendRate;
 
    std::mutex mtx_1;
    std::mutex mtx_2;

    bool m_bSonar_; 
    bool m_bExit_;
    bool m_bExit_2;
    
    CUart *m_pUart;
    CRobotSerial *m_pSerial;
};
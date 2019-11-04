/**********************************************************************************
** MIT License
** 
** Copyright (c) 2019 I-Quotient-Robotics https://github.com/I-Quotient-Robotics
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** 
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
** 
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*********************************************************************************/

#include "BerserkerDriver.h"

#define SLEEP_TIME_MS 10
#define MAX_VX 2.0
#define MAX_VTH 3.14159

static inline void delay()
{
#if defined(_WIN32)
  Sleep(SLEEP_TIME_MS);
#else
  usleep(SLEEP_TIME_MS * 1000);
#endif
}

IQR::BerserkerDriver::BerserkerDriver(const std::string &portName)
    : _readFlage(false)
{
  _com = new QSerialPort(portName,
                         QSerialPort::Baud115200,
                         QSerialPort::Data8,
                         QSerialPort::OneStop,
                         QSerialPort::NoParity,
                         QSerialPort::NoFlowControl);
  _com->open(QSerialPort::ReadAndWrite);
  if (_com->getError())
  {
    std::cout << "open " << portName << " serial port error code:" << _com->getError() << std::endl;
  }
  _frame = new QSerialFrame(0x01);

  start();
}

IQR::BerserkerDriver::~BerserkerDriver()
{
  stop();
  delete _com;
  delete _frame;
}

void IQR::BerserkerDriver::getEncoder(int32_t &LE, int32_t &RE)
{
  std::lock_guard<std::mutex> lck(_mtx);
  LE = _encoder.EL;
  RE = _encoder.ER;
}

void IQR::BerserkerDriver::getWheelSpeed(float &VL, float &VR)
{
  std::lock_guard<std::mutex> lck(_mtx);
  VL = _encoder.vL;
  VR = _encoder.vR;
}

void IQR::BerserkerDriver::setSpeed(const float &vx, const float &vth)
{
  if (_com->isOpen())
  {
    if (abs(vx) > MAX_VX || abs(vth) > MAX_VTH)
      return;
      
    byte msg[10] = {
        0,
    };
    msg[0] = 0x01;
    msg[1] = 0x00;
    QSerialFrame::float32ToByte(vx, msg + 2);
    QSerialFrame::float32ToByte(vth, msg + 6);

    std::lock_guard<std::mutex> lck(_mtx);
    _frame->BuildFrame(msg, 10);
    _com->write(_frame->SenData, _frame->SenData_Len);
  }
  else
  {
  }
}

void IQR::BerserkerDriver::run()
{
   _readFlage = true;
  byte buf[32] = {
      0,
  };
  int32_t date_len = 0;
  while ( _readFlage)
  {
    _mtx.lock();
    date_len = _com->read(buf, 32);
    //std::cout << date_len << "\n";
    for (int32_t i = 0; i < date_len; i++)
    {
      //std::cout << i << "\n";
      if (_frame->ReadFrame(buf[i]))
      {
        switch (_frame->RcvData[0])
        {
        case 0x01:
          _encoder.dL = QSerialFrame::bytesToInt32(_frame->RcvData + 2);
          _encoder.dR = QSerialFrame::bytesToInt32(_frame->RcvData + 6);
          _encoder.dt = QSerialFrame::bytesToInt32(_frame->RcvData + 10);
          _encoder.vL = QSerialFrame::byteToFloat32(_frame->RcvData + 14);
          _encoder.vR = QSerialFrame::byteToFloat32(_frame->RcvData + 18);
          _encoder.EL = QSerialFrame::bytesToInt32(_frame->RcvData + 22);
          _encoder.ER = QSerialFrame::bytesToInt32(_frame->RcvData + 26);
          break;
        case 0x02:
          break;
        default:
          break;
        }
      }
    }
    _mtx.unlock();
    delay();
  }
}

inline void IQR::BerserkerDriver::stop()
{
  std::lock_guard<std::mutex> lck(_mtx);
   _readFlage = false;
  //join();
}

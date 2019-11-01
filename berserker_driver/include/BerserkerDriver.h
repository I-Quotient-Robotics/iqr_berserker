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

#ifndef BERSERKERDRIVER_H
#define BERSERKERDRIVER_H

#include <iostream>
#include <sstream>
#include <mutex>
#include "QSerialFrame.h"
#include "QSerialPort.h"
#include "QThread.h"

namespace IQR
{

struct BerserkerEncoder
{
  int32_t dL;
  int32_t dR;
  uint32_t dt;
  float vL;
  float vR;
  int32_t EL;
  int32_t ER;
};


class BerserkerDriver : public QThread
{

public:
  BerserkerDriver(const std::string &portName);
  ~BerserkerDriver();

  void getEncoder(float &LE, float &RE);
  void getWheelSpeed(float &VL, float &VR);
  void setSpeed(const float &vx, const float &vth);

protected:
  void run();
  inline void stop();

private:
  bool _readFlage;
  std::mutex _mtx;
  volatile BerserkerEncoder _encoder;

  QSerialPort *_com;
  QSerialFrame *_frame;
};
} // namespace IQR

#endif //BERSERKERDRIVER_H
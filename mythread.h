#ifndef MYTHREAD_H
#define MYTHREAD_H

#include <QThread>
#include <QWaitCondition>
#include <QtSerialPort/QSerialPort>

enum {Manual, RglTp_WithOnePID, RglTp_WithTwoPID, RglTp_PD_I, OneComPID};

class MyThread : public QThread
{
  Q_OBJECT
public:
  //void mystart(int fromButton);
  void run() override;

  int mRegulatorType;

signals:
  void response(const int16_t* dta);

private:
  QWaitCondition cond;
  double mDta[8];
  int16_t mBfRc[8];
  int16_t mBfTr[8];
//FOR 1 PID:
  float PID_1out;
//for 2PID:
  float Vel_Des;
  float PID_PWM;
  float last_1PID;
  float err;
  float fIntgr, frdiff,pid;
  float itg, diff;
  float error, last_err = 0.0;

  float inert(float &out, float inp, float K, float T, float dt);
  float rdiff(float &out, float dinp, float Td, float Tf, float dt);
  float intgr(float &out, float inp, float T, float dt);
  float saturate(float &inp, float fmn, float fmx);
  float update1PID(float pgain, float igain, float dgain, float fgain, float target, float cur, float deltaTime);
// Public access to gain parameters
  //

};

#endif // MYTHREAD_H

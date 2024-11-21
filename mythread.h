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
  //

};

#endif // MYTHREAD_H

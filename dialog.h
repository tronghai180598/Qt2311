#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "qcustomplot.h"
#include "mythread.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Dialog; }
QT_END_NAMESPACE

class Dialog : public QDialog
{
  Q_OBJECT
  void myResponse(const int16_t* dta);

public:
  Dialog(QWidget *parent = nullptr);
  ~Dialog();

  int mCurrMethod;
// FOR MANUAL:
  int PWM_manual;
  int PID_PWM_manual;
// FOR 1 PID:
  int PWM_1PID;
  float Kp, Ti, Td, Tf, Des_Ang;
// FOR 2 PID:
  int PWM_2PID;
  float Kp_ang, Ki_ang, Kd_ang, Des_Ang_2;
  float Kp_vel, Ki_vel, Kd_vel;
// FOR 1 CM PID:
  float Kpc;
  float Kic;
  float Kdc;
  float Des_Ang_cm;
  int PWM_CM_PID;

private slots:
  void on_tabWidget_currentChanged(int index);
  void on_horizontalSlider_valueChanged(int value);
  void on_horizontalSlider_2_valueChanged(int value);
  void on_horizontalSlider_3_valueChanged(int value);
  void on_horizontalSlider_4_valueChanged(int value);
  void on_horizontalSlider_7_valueChanged(int value);
  void on_btl_start_clicked();
  void on_btl_send_clicked();

private:
  Ui::Dialog *ui;
  QCustomPlot *mChart;
  MyThread *mThread;
};
#endif // DIALOG_H

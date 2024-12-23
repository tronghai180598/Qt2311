//Da them 1 nhanh
//Hai da them
#include "mythread.h"
#include <inttypes.h>
#include <stdio.h>
#include "dialog.h"
#include <QTime>
//#include "controller.h"
#include <QDebug>

float MyThread::inert(float& out, float inp, float K, float T, float dt) {
    out += dt * (inp * K - out) / T;
    return out;
}
float MyThread::rdiff(float& out, float dinp, float Td, float Tf, float dt) {
    out = out * (1 - dt / Tf) + dinp * Td / Tf;
    return out;
}

float MyThread::intgr(float& out, float inp, float T, float dt) {
    out += dt * inp / T;
    return out;
}
float MyThread::saturate(float& inp, float fmn, float fmx) {
    if (inp < fmn) inp = fmn;
    if (inp > fmx) inp = fmx;
    return inp;
}
float MyThread::update1PID(float pgain, float igain, float dgain, float fgain, float target, float cur, float deltaTime) {
    err = target - cur;
    err *= pgain;
    fIntgr = intgr(fIntgr, err, igain, deltaTime);
    fIntgr = saturate(fIntgr, -100.0, 100.0);
    frdiff = rdiff(frdiff, (err - last_1PID), dgain, fgain, deltaTime);
    pid = inert(pid, (err + fIntgr + frdiff), 1.0, 0.03, deltaTime);
    last_1PID = err;
    pid = saturate(pid, -100.0, 100.0);
    return pid;
}

void MyThread::run()
{
    int cnt = 1;
    Dialog* pDlg = ((Dialog*) parent());
    QSerialPort* sPort = new QSerialPort();
    int strtTmMs = QTime::currentTime().msecsSinceStartOfDay();
    sPort->close();
    sPort->setPortName("/dev/ttyUSB0");
    if(sPort->open(QIODevice::ReadWrite)){
      printf("sPort is opened\n");
    }
    else {
      printf("sPort isnt opened\n");
    }
    sPort->setBaudRate(QSerialPort::Baud115200);
    sPort->setFlowControl(QSerialPort::NoFlowControl);
    sPort->setParity(QSerialPort::NoParity);
    sPort->setDataBits(QSerialPort::Data8);
    sPort->setStopBits(QSerialPort::OneStop);
    int oldTmMs = strtTmMs;
    int rspTmMs = strtTmMs;
    int inrt1 = 0, inrt2 = 0;

    while (1) {
        if (sPort->waitForReadyRead(-1)) {  // Use a timeout to allow thread closure
            QByteArray requestData = sPort->readAll();
            if (requestData.size() > 3) {
                // Ensure proper size and copy data
                memcpy(mBfRc, requestData.data(), 4);
                int crntTmMs = QTime::currentTime().msecsSinceStartOfDay();

                uint32_t elapsed_ms = (crntTmMs - strtTmMs) & 0x0ffffffff;
                memcpy(&mBfRc[2], &elapsed_ms, sizeof(elapsed_ms));

                mBfRc[4] = crntTmMs - oldTmMs;
                oldTmMs = crntTmMs;
                mRegulatorType = pDlg->mCurrMethod;
                switch (mRegulatorType) {
                    case 0: // Manual
                    {
                        uint32_t pwmValue = ((pDlg->PWM_manual) + (pDlg->PID_PWM_manual)) | (((pDlg->PWM_manual) - (pDlg->PID_PWM_manual)) << 16);
                        memcpy(mBfTr, &pwmValue, sizeof(pwmValue));
                        qDebug() << "Manual Control: mBfTr[0]: " << mBfTr[0] << " mBfTr[1]: " << mBfTr[1];
                        break;
                    }
                    case 1:  // 1 PID Control
                    {
                        // Block scope for case 1
//                        pid = Controller(pDlg->Kp, pDlg->Ti, pDlg->Td, pDlg->Tf);
                        PID_1out = update1PID(pDlg->Kp, pDlg->Ti, pDlg->Td, pDlg->Tf, pDlg->Des_Ang, mBfRc[0], mBfRc[4] * 0.001);
                        int pid_output = static_cast<int>(PID_1out);
                        uint32_t pwmValue =  ((pDlg->PWM_1PID) + pid_output) | (((pDlg->PWM_1PID) - pid_output) << 16);
                        memcpy(mBfTr, &pwmValue, sizeof(pwmValue));
                        qDebug() << "1 PID Control - PID_out: " << pid_output;
                        break;
                    }
//                    case 1:  // 1 PID Control
//                    {
//                        // Block scope for case 1
//                        Controller PID_1 = Controller(pDlg->Kp, pDlg->Ti, pDlg->Td, pDlg->Tf);
//                        PID_1out = PID_1.update1PID(pDlg->Des_Ang, mBfRc[0], mBfRc[4] * 0.001);
//                        int pid_output = static_cast<int>(PID_1out);
//                        uint32_t pwmValue =  ((pDlg->PWM_1PID) + pid_output) | (((pDlg->PWM_1PID) - pid_output) << 16);
//                        memcpy(mBfTr, &pwmValue, sizeof(pwmValue));
//                        qDebug() << "1 PID Control - PID_out: " << pid_output;
//                        break;
//                    }
                    case 2:  // 2 PID Control
                    {
                        // Block scope for case 2
//                        Controller Acc_PID = Controller(pDlg->Kp_ang, pDlg->Ki_ang, pDlg->Kd_ang, 0);
//                        Controller Vel_PID = Controller(pDlg->Kp_vel, pDlg->Ki_vel, pDlg->Kd_vel, 0);
//                        Vel_Des = Acc_PID.update2PID(pDlg->Des_Ang_2, mBfRc[0], mBfRc[4] * 0.001);
//                        PID_PWM = Vel_PID.update2PID(Vel_Des, mBfRc[1], mBfRc[4] * 0.001);
//                        int pid_out = static_cast<int>(PID_PWM);

//                        uint32_t pwmValue =  ((pDlg->PWM_2PID) + pid_out) | (((pDlg->PWM_2PID) - pid_out) << 16);
//                        memcpy(mBfTr, &pwmValue, sizeof(pwmValue));
//                        qDebug() << "2 PID Control - PID_out: " << pid_out;
                        break;
                    }
                    case 3:
                        // Logic for regulator type 3
                        break;
                    case 4:
                {
                        error = pDlg->Des_Ang_cm - mBfRc[0];
                        qDebug() << "ERROR: " << error;
                        if (-10<error<10){
                            itg += error *mBfRc[4] * 0.001;
                            itg = saturate(itg, -200.0, 200.0);
                        }

                        diff = (error - last_err)/(mBfRc[4] * 0.001);
                        pid  = pDlg->Kpc*error + pDlg->Kic*itg + pDlg->Kdc*diff;
                        pid = saturate(pid, -200.0, 200.0);
                        last_err = error ;
                        int pid_out = static_cast<int>(pid);
                        uint32_t pwmValue =  ((pDlg->PWM_CM_PID) + pid_out) | (((pDlg->PWM_CM_PID) - pid_out) << 16);
                        memcpy(mBfTr, &pwmValue, sizeof(pwmValue));
                        qDebug() << "1 PID Control - PID_out: " << pid;
                        break;
                }
                    default:
                        qWarning("Invalid regulator type");
                        break;
                }
                // Write to Arduino
                sPort->write((const char*)mBfTr, 4);

                // Calculate average for chart message every 50 ms
                inrt1 += mBfRc[0];
                inrt2 += mBfRc[1];
                if ((crntTmMs - rspTmMs) > 50) {
                    mBfRc[0] = inrt1 / cnt; inrt1 = 0;
                    mBfRc[1] = inrt2 / cnt; inrt2 = 0;
                    emit response(mBfRc);
                    rspTmMs = crntTmMs;
                    cnt = 1;
                } else cnt++;
            }
        } else {
            msleep(1);  // Yield control if no data
        }
    }
return;
}

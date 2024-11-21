//Da kiem tra. OK
//Da kiem tra Lan 2
#include "controller.h"
#include <QDebug>

Controller::Controller(){
  pgain = 0;
  igain = 0;
  dgain = 0;
  fgain = 0;
}
// Parameterized constructor
Controller::Controller(float p, float i, float d, float f)
 {
    Controller();
    pgain = p;
    igain = i;
    dgain = d;
    fgain = f;
}
float Controller::inert(float& out, float inp, float K, float T, float dt) {
    out += dt * (inp * K - out) / T;
    return out;
}

float Controller::rdiff(float& out, float dinp, float Td, float Tf, float dt) {
    out = out * (1 - dt / Tf) + dinp * Td / Tf;
    return out;
}

float Controller::intgr(float& out, float inp, float T, float dt) {
    out += dt * inp / T;
    return out;
}
float Controller::saturate(float& inp, float fmn, float fmx) {
    if (inp < fmn) inp = fmn;
    if (inp > fmx) inp = fmx;
    return inp;
}
float Controller::update1PID(float target, float cur, float deltaTime) {
    err = target - cur;
    err *= pgain;
    fIntgr = intgr(fIntgr, err, igain, deltaTime);
    fIntgr = saturate(fIntgr, -400.0, 400.0);
    frdiff = rdiff(frdiff, (err - last_1PID), dgain, fgain, deltaTime);
    pid = inert(pid, (err + fIntgr + frdiff), 1.0, 0.035, deltaTime);
    last_1PID = err;
    pid = saturate(pid, -400.0, 400.0);
    return pid;
}
float Controller::update2PID(float target, float cur, float deltaTime) {
    err = target - cur;
    pTerm = pgain * err;
    iState += err * deltaTime;
    iState = saturate(iState, -400.0, 400.0);
    iTerm = igain * iState;
    dTerm = (dgain * (err - last_2PID)) / deltaTime;
    fpid = pTerm + iTerm + dTerm;
    last_2PID = err;
    fpid = saturate(fpid, -400.0, 400.0);
    return fpid;
}
//float Controller::update1PD_I(float target_acc, float cur_acc, float cur_gyro, float deltaTime){
//    err_acc = target_acc - cur_acc;
//    P_a = pgain * err;
//    D_a = dgain * cur_gyro;
//    Gyro_des = P_a - D_a;
//    err_gyro = Gyro_des - cur_gyro;
//    P_v =
//}

//void Controller::resetError() {
//    iState = 0.0f;
//}

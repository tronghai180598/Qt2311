#ifndef CONTROLLER_H
#define CONTROLLER_H


class Controller
{
public:
    Controller();
    Controller(float p, float i, float d, float f);

    float update1PID(float target, float cur, float deltaTime);
    float update2PID(float target, float cur, float deltaTime);
//    float update1PD_I(float target, float cur, float cur_gyro, float deltaTime);
//    void resetError();

private:
    float pgain; // Proportional gain
    float igain; // Integral gain
    float dgain; // Derivative gain
    float fgain; // Filter gain
    float last_1PID;
    float err;

    float pid, fpid;
    float last_2PID;
    float pTerm, iTerm, dTerm;
    float iState;
//float 1PID:
    float fIntgr, frdiff;
    float P_a, D_a, P_v, D_v;
    float Gyro_des, err_acc, err_gyro;
    float inert(float &out, float inp, float K, float T, float dt);
    float rdiff(float &out, float dinp, float Td, float Tf, float dt);
    float intgr(float &out, float inp, float T, float dt);
    float saturate(float &inp, float fmn, float fmx);
  // Public access to gain parameters
};

#endif // CONTROLLER_H

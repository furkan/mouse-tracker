#include<ansi_c.h>
#include<math.h>
#include<windows.h>
#include<time.h>

void kalman(float cursor_pos[], float actual[], float noisy_curr[], float noise) {
	
	float R[4] = {0.2845,0.0045,0.0045,0.0455}; // 2x2
    float Q[4] = {0.01,0.01,0.01,0.01}; // 4x4 diagonal
    float P[16] = {100,0,0,0,0,100,0,0,0,0,100,0,0,0,0,100}; // 4x4
    float Ppre[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // 4x4
    float dt = 1;
    float U[4] = {0,0,0,0}; // temporary matrix
    float a[4] = {0,0,0,0}; //  K = y A'(AA')^-1   // a = A
    float b[4] = {0,0,0,0}; //  u =   A'(AA')^-1   // b = (AA')^-1
    float c = 0;
    float K[8] = {0,0,0,0,0,0,0,0}; // KALMAN GAIN
    float predicted[4]={0,0,0,0}; // INITIAL PREDICTION ????  (SANIRIM Xpos,Ypos,Xvelo,Yvelo diye dizli ve bunu ?l??mden al?yoruz)
    float noisy_currX, noisy_currY;
	float curr_x = cursor_pos[0];
	float curr_y = cursor_pos[1];
	
	noisy_currX = curr_x + noise*(rand()%50 - rand()%50);
    noisy_currY = curr_y + noise*(rand()%50 - rand()%50);

    noisy_curr[0] = noisy_currX;
    noisy_curr[1] = noisy_currY;

    predicted[0] = actual[0] + dt * actual[2];
    predicted[1] = actual[1] + dt * actual[3];
    predicted[2] = actual[2];
    predicted[3] = actual[3];

    Ppre[0]  = P[0]  + P[8] *dt + dt*(P[2]+P[10]*dt)  + Q[0];
    Ppre[1]  = P[1]  + P[9] *dt + dt*(P[3]+P[11]*dt);
    Ppre[2]  = P[2]  + P[10]*dt;
    Ppre[3]  = P[3]  + P[11]*dt;
    Ppre[4]  = P[4]  + P[12]*dt + dt*(P[6]+P[14]*dt);
    Ppre[5]  = P[5]  + P[13]*dt + dt*(P[7]+P[15]*dt)  + Q[1];
    Ppre[6]  = P[6]  + P[14]*dt;
    Ppre[7]  = P[7]  + P[15]*dt;
    Ppre[8]  = P[8]  + P[10]*dt;
    Ppre[9]  = P[9]  + P[11]*dt;
    Ppre[10] = P[10]                                  + Q[2];
    Ppre[11] = P[11];
    Ppre[12] = P[12] + P[14]*dt;
    Ppre[13] = P[13] + P[15]*dt;
    Ppre[14] = P[14];
    Ppre[15] = P[15]                                  + Q[3];

    a[0] = Ppre[0] + R[0];
    a[1] = Ppre[1] + R[1];
    a[2] = Ppre[4] + R[2];
    a[3] = Ppre[5] + R[3];

    c = (a[0]*a[0]+a[1]*a[1])*(a[2]*a[2]+a[3]*a[3]) - (a[0]*a[2]+a[1]*a[3])*(a[0]*a[2]+a[1]*a[3]);

    b[0] =  (a[2]*a[2]+a[3]*a[3]) / c;
    b[1] = -(a[0]*a[2]+a[1]*a[3]) / c;
    b[2] = -(a[0]*a[2]+a[1]*a[3]) / c;
    b[3] =  (a[0]*a[0]+a[1]*a[1]) / c;

    U[0] = a[0] * b[0] + a[2] * b[2];
    U[1] = a[0] * b[1] + a[2] * b[3];
    U[2] = a[1] * b[0] + a[3] * b[2];
    U[3] = a[1] * b[1] + a[3] * b[3];

    K[0] = Ppre[0] *U[0] + Ppre[1] *U[2];
    K[1] = Ppre[0] *U[1] + Ppre[1] *U[3];
    K[2] = Ppre[4] *U[0] + Ppre[5] *U[2];
    K[3] = Ppre[4] *U[1] + Ppre[5] *U[3];
    K[4] = Ppre[8] *U[0] + Ppre[9] *U[2];
    K[5] = Ppre[8] *U[1] + Ppre[9] *U[3];
    K[6] = Ppre[12]*U[0] + Ppre[13]*U[2];
    K[7] = Ppre[12]*U[1] + Ppre[13]*U[3];

    actual[0] = predicted[0]+K[0]*(noisy_currX-predicted[0])+K[1]*(noisy_currY-predicted[1]);
    actual[1] = predicted[1]+K[2]*(noisy_currX-predicted[0])+K[3]*(noisy_currY-predicted[1]);
    actual[2] = predicted[2]+K[4]*(noisy_currX-predicted[0])+K[5]*(noisy_currY-predicted[1]);
    actual[3] = predicted[3]+K[6]*(noisy_currX-predicted[0])+K[7]*(noisy_currY-predicted[1]);

    P[0]  =  Ppre[0]*(1-K[0])-Ppre[4]*K[1];
    P[1]  =  Ppre[1]*(1-K[0])-Ppre[5]*K[1];
    P[2]  =  Ppre[2]*(1-K[0])-Ppre[6]*K[1];
    P[3]  =  Ppre[3]*(1-K[0])-Ppre[7]*K[1];
    P[4]  = -Ppre[0]*K[2]+Ppre[4]*(1-K[3]);
    P[5]  = -Ppre[1]*K[2]+Ppre[5]*(1-K[3]);
    P[6]  = -Ppre[2]*K[2]+Ppre[6]*(1-K[3]);
    P[7]  = -Ppre[3]*K[2]+Ppre[7]*(1-K[3]);
    P[8]  = -Ppre[0]*K[4]-Ppre[4]*K[5]+Ppre[8];
    P[9]  = -Ppre[1]*K[4]-Ppre[5]*K[5]+Ppre[9];
    P[10] = -Ppre[2]*K[4]-Ppre[6]*K[5]+Ppre[10];
    P[11] = -Ppre[3]*K[4]-Ppre[7]*K[5]+Ppre[11];
    P[12] = -Ppre[0]*K[6]-Ppre[4]*K[7]+Ppre[12];
    P[13] = -Ppre[1]*K[6]-Ppre[5]*K[7]+Ppre[13];
    P[14] = -Ppre[2]*K[6]-Ppre[6]*K[7]+Ppre[14];
    P[15] = -Ppre[3]*K[6]-Ppre[7]*K[7]+Ppre[15];	
}
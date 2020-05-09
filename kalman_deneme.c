#include<stdio.h>
#include<math.h>
#include<windows.h>
#include<time.h>

int main()
{
/* Initialize the Kalman filter
 R - measurement noise, (2x2): R=[[0.2845,0.0045]',[0.0045,0.0455]'];
 H - transform from measure to state (2x4): H=[[1,0]',[0,1]',[0,0]',[0,0]'];
 Q - system noise, (4x4) (diagonal): Q=0.01*eye(4);
 P - the status covarince matrix (4x4) (diagonal): P=100*eye(4);
 A - state transform matrix (4x4): A=[[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];
*/


// NOTE THAT MATRICES ARE CONSTRUCTED IN THE ROW ORDER; THAT IS, IF MATRIS IS 4X4 THAN THE ARRAY CONTAINS 16 ELEMENTS AND FIRST 4 BELONGS TO FIRST ROW
// SECOND 4 BELONGS TO SECOND ROW ETC


float R[4] = {0.2845,0.0045,0.0045,0.0455};
// float H[8] = {1,0,0,0,0,1,0,0}; SADECE 1 VE 0 OLDUÐU ÝÇÝN KAÐIDA ÇIKARTTIM ÇARPIM SONUÇLARINI O YÜZDEN BUNU KULLANMIYORUZ
float Q[4] = {0.01,0.01,0.01,0.01};
float P[16] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
float Ppre[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float dt = 1;
// float A[16] ÇARPIM SONUCU BUNU DA ÇIKARDIK O YÜZDEN BUNU TANIMLAMAYA GEREK YOK
float U[4] = {0,0,0,0}; // GEÇÝCÝ BÝR MATRÝS BU
float det = 0;
float K[8] = {0,0,0,0,0,0,0,0}; // KALMAN GAIN
float predicted[4]={0,0,0,0}; // INITIAL PREDICT ????  (SANIRIM Xpos,Ypos,Xvelo,Yvelo diye dizli ve bunu ölçümden alýyoruz)
float actual[4]={0,0,0,0}; // KALMAN SONUCU
float pre_x, pre_y, vel_x, vel_y, curr_x, curr_y, noisy_preX, noisy_preY, noisy_currX, noisy_currY, noisy_velX, noisy_velY;
float time_pass = 0, t;
float noise = 0.1;
// float temp1,temp2,temp3,temp4;

/*
  Ppre = A*P*A' + Q;
  K = Ppre*H'/(H*Ppre*H'+R);
  actual(i,:) = (predicted + K*([centroidx(i),centroidy(i)]' - H*predicted))';
  P = (eye(4)-K*H)*Ppre;
*/

POINT punkt;

    while(1)
    {
        t = clock();
        GetCursorPos(&punkt);
        pre_x = punkt.x; // bunlara noise eklenebilir ???
        pre_y = punkt.y;
        system("cls");

        noisy_preX = pre_x + noise*(rand()%100);
        noisy_preY = pre_y + noise*(rand()%100);

        GetCursorPos(&punkt);
        curr_x = punkt.x;
        curr_y = punkt.y;
        system("cls");
        t = clock()-t;
        time_pass = t/CLOCKS_PER_SEC;

        noisy_currX = curr_x + noise*(rand()%50);
        noisy_currY = curr_y + noise*(rand()%50);

        vel_x = (noisy_currX-noisy_preX)/time_pass;
        vel_y = (noisy_currY-noisy_preY)/time_pass;

        predicted[0] = noisy_currX;
        predicted[1] = noisy_currY;
        predicted[2] = vel_x;
        predicted[3] = vel_y;

        Ppre[0] = P[0]+dt*P[8]+dt*(P[2]+P[10]*dt)+Q[0];
        Ppre[1] = P[1]+dt*P[9]+dt*(P[3]+P[11]*dt);
        Ppre[2] = P[2]+P[10]*dt;
        Ppre[3] = P[3]+P[11]*dt;
        Ppre[4] = P[4]+dt*P[12]+dt*(P[6]+P[14]*dt);
        Ppre[5] = P[5]+dt*P[13]+dt*(P[7]+P[15]*dt)+Q[1];
        Ppre[6] = P[6]+P[14]*dt;
        Ppre[7] = P[7]+P[15]*dt;
        Ppre[8] = P[8]+P[10]*dt;
        Ppre[9] = P[9]+P[11]*dt;
        Ppre[10] = P[10]+Q[2];
        Ppre[11] = P[11];
        Ppre[12] = P[12]+P[14]*dt;
        Ppre[13] = P[13]+P[15]*dt;
        Ppre[14] = P[14];
        Ppre[15] = P[15]+Q[3];

        /*
        temp1 = P[0]+dt*P[8];;
        temp2 = dt*(P[2]+P[10]*dt)+Q[0];
        temp3 = (Ppre[4]+R[2]);
        temp4 = (Ppre[4]+R[2])*(Ppre[1]+R[1]);
        */

        det = 1/((Ppre[0]+R[0])*(Ppre[5]+R[3])-(Ppre[4]+R[2])*(Ppre[1]+R[1]));
        U[0] = (Ppre[5]+R[3])/det;
        U[1] = -(Ppre[1]+R[1])/det;
        U[2] = -(Ppre[4]+R[2])/det;
        U[3] = (Ppre[0]+R[0])/det;

        K[0] = Ppre[0]*U[0]+Ppre[1]*U[2];
        K[1] = Ppre[0]*U[1]+Ppre[1]*U[3];
        K[2] = Ppre[4]*U[0]+Ppre[5]*U[2];
        K[3] = Ppre[4]*U[1]+Ppre[5]*U[3];
        K[4] = Ppre[8]*U[0]+Ppre[9]*U[2];
        K[5] = Ppre[8]*U[1]+Ppre[9]*U[3];
        K[6] = Ppre[12]*U[0]+Ppre[13]*U[2];
        K[7] = Ppre[12]*U[1]+Ppre[13]*U[3];

        actual[0] = predicted[0]+K[0]*(curr_x-predicted[0])+K[1]*(curr_y-predicted[1]);
        actual[1] = predicted[1]+K[2]*(curr_x-predicted[0])+K[3]*(curr_y-predicted[1]);
        actual[2] = predicted[2]+K[4]*(curr_x-predicted[0])+K[5]*(curr_y-predicted[1]);
        actual[3] = predicted[3]+K[6]*(curr_x-predicted[0])+K[7]*(curr_y-predicted[1]);

        printf("ACTUAL = %f, %f\n\n",curr_x,curr_y);
        printf("KALMAN = %f, %f\n\n",actual[0],actual[1]);

        P[0] = Ppre[0]*(1-K[0])-Ppre[4]*K[1];
        P[1] = Ppre[1]*(1-K[0])-Ppre[5]*K[1];
        P[2] = Ppre[2]*(1-K[0])-Ppre[6]*K[1];
        P[3] = Ppre[3]*(1-K[0])-Ppre[7]*K[1];
        P[4] = -Ppre[0]*K[2]+Ppre[4]*(1-K[3]);
        P[5] = -Ppre[1]*K[2]+Ppre[5]*(1-K[3]);
        P[6] = -Ppre[2]*K[2]+Ppre[6]*(1-K[3]);
        P[7] = -Ppre[3]*K[2]+Ppre[7]*(1-K[3]);
        P[8] = -Ppre[0]*K[4]-Ppre[4]*K[5]+Ppre[8];
        P[9] = -Ppre[1]*K[4]-Ppre[5]*K[5]+Ppre[9];
        P[10] = -Ppre[2]*K[4]-Ppre[6]*K[5]+Ppre[10];
        P[11] = -Ppre[3]*K[4]-Ppre[7]*K[5]+Ppre[11];
        P[12] = -Ppre[0]*K[6]-Ppre[4]*K[7]+Ppre[12];
        P[13] = -Ppre[1]*K[6]-Ppre[5]*K[7]+Ppre[13];
        P[14] = -Ppre[2]*K[6]-Ppre[6]*K[7]+Ppre[14];
        P[15] = -Ppre[3]*K[6]-Ppre[7]*K[7]+Ppre[15];

    }

}

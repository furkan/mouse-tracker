#include<stdio.h>
#include<math.h>

float **addition44(float m[4][4], float n[4][4]); // 4x4 + 4x4
float **addition22(float m[2][2], float n[2][2]); // 2x2 + 2x2
float **addition21(float m[2][1], float n[2][1]); // 2x1 + 2x1

float **multiply4444(float m[4][4], float n[4][4]); // 4x4 * 4x4
float **multiply4224(float m[4][2], float n[2][4]); // 4x2 * 2x4
float **multiply4442(float m[4][4], float n[4][2]); // 4x4 * 4x2
float **multiply2442(float m[2][4], float n[4][2]); // 2x4 * 4x2
float **multiply2441(float m[2][4], float n[4][1]); // 2x4 * 4x1
float **multiply4221(float m[4][2], float n[2][1]); // 4x2 * 2x1

float **transpose44(float m[4][4]); // transpose of 4x4
float **transpose24(float m[2][4]); // transpose of 2x4
float **transpose21(float m[2][1]); // transpose of 2x1
float **transpose14(float m[1][4]); // transpose of 1x4

float **inverse(float m[2][2]);

int main()
{
/* Initialize the Kalman filter
 R - measurement noise, (2x2): R=[[0.2845,0.0045]',[0.0045,0.0455]'];
 H - transform from measure to state (2x4): H=[[1,0]',[0,1]',[0,0]',[0,0]'];
 Q - system noise, (4x4) (diagonal): Q=0.01*eye(4);
 P - the status covarince matrix (4x4) (diagonal): P=100*eye(4);
 A - state transform matrix (4x4): A=[[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];
*/

float R[4] = {0.2845,0.0045,0.0045,0.0455}; // R11, R21, R12, R22
int H[8] = {1,0,0,1,0,0,0,0}; // H11 H21 H12 H22 H13 H23 H14 H24
float Q[4] = {0.01,0.01,0.01,0.01}; // Q11 Q22 Q33 Q44
float P[16] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}; // Ilk 4 ilk column, ikinici 4 ikinci column ...
int dt=1;
int A[4] = {1,1,dt,1}; // All necessary Elements of A are stored
int measured_x=0, estimated_x=0;;
int measured_y=0, estimated_y=0;
float velocity_x=0, velocity_est_x=0;
float velocity_y=0, velocity_est_y=0;
float measurement_vector[4]={measured_x,measured_y,velocity_x,velocity_y};
float estimation_vector[4]={estimated_x,estimated_y,velocity_est_x,velocity_est_y};

float temp[2][2]={1,2,3,4};
float **a;
a = inverse(temp);



}


////ADDITION//////////////////ADDITION///////////
///////////ADDITION/////////////ADDITION/////////
////////////////////////////////////////////////
float **addition44(float m[4][4], float n[4][4])
{
    float **sum;
    int i, j;

    //Add both matrices
    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            sum[i][j] = m[i][j] + n[i][j];
        }
    }

    //print resultant matrix
    printf("\nAddition of both Matrix is:\n");

    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            printf("%d\t",sum[i][j]);
        }
        printf("\n");
    }

    return sum;
}

float **addition22(float m[2][2], float n[2][2])
{
    float **sum;
    int i, j;

    //Add both matrices
    for(i=0;i<2;i++)
    {
        for(j=0;j<2;j++)
        {
            sum[i][j] = m[i][j] + n[i][j];
        }
    }

    //print resultant matrix
    printf("\nAddition of both Matrix is:\n");

    for(i=0;i<2;i++)
    {
        for(j=0;j<2;j++)
        {
            printf("%d\t",sum[i][j]);
        }
        printf("\n");
    }

    return sum;
}

float **addition21(float m[2][1], float n[2][1])
{
    float **sum;
    int i, j;

    //Add both matrices
    for(i=0;i<2;i++)
    {
        for(j=0;j<1;j++)
        {
            sum[i][j] = m[i][j] + n[i][j];
        }
    }

    //print resultant matrix
    printf("\nAddition of both Matrix is:\n");

    for(i=0;i<2;i++)
    {
        for(j=0;j<1;j++)
        {
            printf("%d\t",sum[i][j]);
        }
        printf("\n");
    }

    return sum;
}



////MULTIPLICATION//////////////////MULTIPLICATION///////////
/////////// MULTIPLICATION/////////////MULTIPLICATION////////
/////////////////////////////////////////////////////////////
float **multiply4444(float m[4][4], float n[4][4])
{

    float **multiple, sum;
    int i, j, k;

    //multiply both matrix m and n
    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            int sum=0;
            for(k=0;k<4;k++)
            {
                sum+= m[i][k] * n[k][j];
            }
            multiple[i][j]=sum;
        }
    }


    //print resultent matrix
    printf("\nMultiplication of both Matrix is:\n");

    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            printf("%d\t",multiple[i][j]);
        }
        printf("\n");
    }

    return multiple;

}

float **multiply4224(float m[4][2], float n[2][4])
{

    float **multiple, sum;
    int i, j, k;

    //multiply both matrix m and n
    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            int sum=0;
            for(k=0;k<2;k++)
            {
                sum+= m[i][k] * n[k][j];
            }
            multiple[i][j]=sum;
        }
    }


    //print resultent matrix
    printf("\nMultiplication of both Matrix is:\n");

    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            printf("%d\t",multiple[i][j]);
        }
        printf("\n");
    }

    return multiple;

}

float **multiply4442(float m[4][4], float n[4][2])
{

    float **multiple, sum;
    int i, j, k;

    //multiply both matrix m and n
    for(i=0;i<4;i++)
    {
        for(j=0;j<2;j++)
        {
            int sum=0;
            for(k=0;k<4;k++)
            {
                sum+= m[i][k] * n[k][j];
            }
            multiple[i][j]=sum;
        }
    }


    //print resultent matrix
    printf("\nMultiplication of both Matrix is:\n");

    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            printf("%d\t",multiple[i][j]);
        }
        printf("\n");
    }

    return multiple;

}

float **multiply2442(float m[2][4], float n[4][2])
{

    float **multiple, sum;
    int i, j, k;

    //multiply both matrix m and n
    for(i=0;i<2;i++)
    {
        for(j=0;j<2;j++)
        {
            int sum=0;
            for(k=0;k<4;k++)
            {
                sum+= m[i][k] * n[k][j];
            }
            multiple[i][j]=sum;
        }
    }


    //print resultent matrix
    printf("\nMultiplication of both Matrix is:\n");

    for(i=0;i<2;i++)
    {
        for(j=0;j<2;j++)
        {
            printf("%d\t",multiple[i][j]);
        }
        printf("\n");
    }

    return multiple;

}

float **multiply2441(float m[2][4], float n[4][1])
{

    float **multiple, sum;
    int i, j, k;

    //multiply both matrix m and n
    for(i=0;i<2;i++)
    {
        for(j=0;j<1;j++)
        {
            int sum=0;
            for(k=0;k<4;k++)
            {
                sum+= m[i][k] * n[k][j];
            }
            multiple[i][j]=sum;
        }
    }


    //print resultent matrix
    printf("\nMultiplication of both Matrix is:\n");

    for(i=0;i<2;i++)
    {
        for(j=0;j<1;j++)
        {
            printf("%d\t",multiple[i][j]);
        }
        printf("\n");
    }

    return multiple;

}


float **multiply4221(float m[4][2], float n[2][1])
{

    float **multiple, sum;
    int i, j, k;

    //multiply both matrix m and n
    for(i=0;i<4;i++)
    {
        for(j=0;j<1;j++)
        {
            int sum=0;
            for(k=0;k<2;k++)
            {
                sum+= m[i][k] * n[k][j];
            }
            multiple[i][j]=sum;
        }
    }


    //print resultent matrix
    printf("\nMultiplication of both Matrix is:\n");

    for(i=0;i<4;i++)
    {
        for(j=0;j<1;j++)
        {
            printf("%d\t",multiple[i][j]);
        }
        printf("\n");
    }

    return multiple;

}

////TRANSPOSE//////////////////TRANSPOSE///////////
/////////// TRANSPOSE/////////////TRANSPOSE////////
///////////////////////////////////////////////////
float **transpose44(float m[4][4])
{
    float **trans;
    int i, j;

    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            trans[i][j]=m[j][i];
        }
    }

    //print Transpose of matrix
    printf("Transpose of Matrix is:\n\n");

    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            printf("%d\t",trans[i][j]);
        }
        printf("\n");
    }

    return trans;
}


float **transpose24(float m[2][4])
{
    float **trans;
    int i, j;

    for(i=0;i<4;i++)
    {
        for(j=0;j<2;j++)
        {
            trans[i][j]=m[j][i];
        }
    }

    //print Transpose of matrix
    printf("Transpose of Matrix is:\n\n");

    for(i=0;i<4;i++)
    {
        for(j=0;j<2;j++)
        {
            printf("%d\t",trans[i][j]);
        }
        printf("\n");
    }

    return trans;
}

float **transpose21(float m[2][1])
{
    float **trans;
    int i, j;

    for(i=0;i<1;i++)
    {
        for(j=0;j<2;j++)
        {
            trans[i][j]=m[j][i];
        }
    }

    //print Transpose of matrix
    printf("Transpose of Matrix is:\n\n");

    for(i=0;i<1;i++)
    {
        for(j=0;j<2;j++)
        {
            printf("%d\t",trans[i][j]);
        }
        printf("\n");
    }

    return trans;
}

float **transpose14(float m[1][4])
{
    float **trans;
    int i, j;

    for(i=0;i<4;i++)
    {
        for(j=0;j<1;j++)
        {
            trans[i][j]=m[j][i];
        }
    }

    //print Transpose of matrix
    printf("Transpose of Matrix is:\n\n");

    for(i=0;i<4;i++)
    {
        for(j=0;j<1;j++)
        {
            printf("%d\t",trans[i][j]);
        }
        printf("\n");
    }

    return trans;
}

////INVERSE//////////////////INVERSE///////////
///////////INVERSE/////////////INVERSE///////
/////////////////////////////////////////////
float **inverse(float mat[2][2]){
    float **a_inverse;
	int i, j;
	float determinant = 0;
	float temp;

		determinant = mat[0][0]*mat[1][1]-mat[0][1]*mat[1][0];

	printf("\n\ndeterminant: %f\n", determinant);

	printf("\nInverse of matrix is: \n");

    temp=mat[0][0];
    mat[0][0]=mat[1][1];
    mat[1][1]=temp;
    mat[0][1]=-mat[0][1];
    mat[1][0]=-mat[1][0];

	for(i = 0; i < 2; i++){
		for(j = 0; j < 2; j++)
            a_inverse[i][j]=mat[i][j]/ determinant;
			printf("%.2f\t",a_inverse[i][j]);

		printf("\n");
	}

   return a_inverse;
}

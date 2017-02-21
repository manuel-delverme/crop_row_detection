//#include "stdafx.h"

#include "Platform.h"

//! #include "nr.h"
//! #include "nrutil.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "RVLConst.h"
#include "RVLWChain.h"
#include "cv.h"
#include "RVLMem.h"
//#include "RVLPtrChain.h"
//#include "RVLMPtrChain.h"
#include "RVLUtil.h"

///////////////////////////////////////////////////////////
//
//     Global variables
//
///////////////////////////////////////////////////////////

// lookUp tables

int sqrt_LookUpTable[65281];
BYTE atan45_LookUpTable[1001];
BYTE log10_LookUpTable[2001];
int chi2_LookUpTable[1201];
double lnFactorial_LookUpTable[101];

/////

CvMat *RVLMatrix31;
CvMat *RVLMatrixHeader31;
CvMat *RVLMatrix33;
CvMat *RVLMatrixHeaderA33;
CvMat *RVLMatrixHeaderB33;
CvMat *RVLMatrixHeaderC33;
double RVLVector3[3];
double RVLMatrixA33[9];
double RVLMatrixB33[9];
double RVLMatrixC33[9];
unsigned char RVLColorMap[] = {
     0,     0,   143,
     0,     0,   159,
     0,     0,   175,
     0,     0,   191,
     0,     0,   207,
     0,     0,   223,
     0,     0,   239,
     0,     0,   255,
     0,    16,   255,
     0,    32,   255,
     0,    48,   255,
     0,    64,   255,
     0,    80,   255,
     0,    96,   255,
     0,   112,   255,
     0,   128,   255,
     0,   143,   255,
     0,   159,   255,
     0,   175,   255,
     0,   191,   255,
     0,   207,   255,
     0,   223,   255,
     0,   239,   255,
     0,   255,   255,
    16,   255,   239,
    32,   255,   223,
    48,   255,   207,
    64,   255,   191,
    80,   255,   175,
    96,   255,   159,
   112,   255,   143,
   128,   255,   128,
   143,   255,   112,
   159,   255,    96,
   175,   255,    80,
   191,   255,    64,
   207,   255,    48,
   223,   255,    32,
   239,   255,    16,
   255,   255,     0,
   255,   239,     0,
   255,   223,     0,
   255,   207,     0,
   255,   191,     0,
   255,   175,     0,
   255,   159,     0,
   255,   143,     0,
   255,   128,     0,
   255,   112,     0,
   255,    96,     0,
   255,    80,     0,
   255,    64,     0,
   255,    48,     0,
   255,    32,     0,
   255,    16,     0,
   255,     0,     0,
   239,     0,     0,
   223,     0,     0,
   207,     0,     0,
   191,     0,     0,
   175,     0,     0,
   159,     0,     0,
   143,     0,     0,
   128,     0,     0};

#ifdef RVL_DEBUG
FILE *fpDebug;
DWORD bDebugFlags;
#endif

///////////////////////////////////////////////////////////

double Max(double *x, int n)
{
	int i;
	double Max;

	Max = x[0];

	for(i = 1; i < n; i++)
		if(x[i] >  Max)
			Max = x[i];

	return Max;
}

double Min(double *x, int n)
{
	int i;
	double Min;

	Min = x[0];

	for(i = 1; i < n; i++)
		if(x[i] <  Min)
			Min = x[i];

	return Min;
}



int Max(int *x, int n)
{
	int Max, i;

	Max = x[0];

	for(i = 1; i < n; i++)
		if(x[i] >  Max)
			Max = x[i];

	return Max;
}

int Min(int *x, int n)
{
	int Min, i;

	Min = x[0];

	for(i = 1; i < n; i++)
		if(x[i] <  Min)
			Min = x[i];

	return Min;
}


double Max(double *x, int n, int &j)
{
	int i;
	double Max;

	Max = x[0];
	j = 0;

	for(i = 1; i < n; i++)
		if(x[i] >  Max)
		{
			j = i;
			Max = x[i];
		}

	return Max;
}

float Max(float *x, int n, int &j)
{
	int i;
	float Max;

	Max = x[0];
	j = 0;

	for(i = 1; i < n; i++)
		if(x[i] >  Max)
		{
			j = i;
			Max = x[i];
		}

	return Max;
}

double Min(double *x, int n, int &j)
{
	int i;
	double Min;

	Min = x[0];
	j = 0;

	for(i = 1; i < n; i++)
		if(x[i] <  Min)
		{
			Min = x[i];
			j = i;
		}

	return Min;
}

float Min(float *x, int n, int &j)
{
	int i;
	float Min;

	Min = x[0];
	j = 0;

	for(i = 1; i < n; i++)
		if(x[i] <  Min)
		{
			Min = x[i];
			j = i;
		}

	return Min;
}



int Max(int *x, int n, int &j)
{
	int Max, i;

	Max = x[0];

	for(i = 1; i < n; i++)
		if(x[i] >  Max)
		{
			Max = x[i];

			j = i;
		}

	return Max;
}

int Min(int *x, int n, int &j)
{
	int Min, i;

	Min = x[0];
	j = 0;

	for(i = 1; i < n; i++)
		if(x[i] <  Min)
		{
			Min = x[i];

			j = i;
		}

	return Min;
}


BOOL TwoEqsTwoVars(double a11, double a12, double a21, double a22, double b1, double b2, double &x1, double &x2)
{
	double tmp = a12 * a21 - a11 * a22;

	if(fabs(tmp) < APPROX_ZERO)
		return FALSE;

	x2 = (a21 * b1 - a11 * b2) / tmp;

	if(fabs(a11) > APPROX_ZERO)
	{
		x1 = (b1 - a12 * x2) / a11;
		return TRUE;
	}
	else if(fabs(a21) > APPROX_ZERO)
	{
		x1 = (b2 - a22 * x2) / a21;
		return TRUE;
	}
	else
		return FALSE;
}

BOOL TwoEqsTwoVars(int a11, int a12, int a21, int a22, int b1, int b2, int &x1, int &x2)
{
	int tmp = a11 * a22 - a12 * a21;

	if(tmp == 0)
		return FALSE;

	x1 = ( a22 * b1 - a12 * b2) / tmp;
	x2 = (-a21 * b1 + a11 * b2) / tmp;

	return TRUE;
}

BOOL TwoEqsTwoVars(int a11, int a12, int a21, int a22, int b1, int b2, double &x1, double &x2)
{
	int tmp = a12 * a21 - a11 * a22;

	if(tmp == 0)
		return FALSE;

	x2 = (double)(a21 * b1 - a11 * b2) / (double)tmp;

	if(a11 != 0)
	{
		x1 = (double)(b1 - a12 * x2) / (double)a11;
		return TRUE;
	}
	else if(a21 != 0)
	{
		x1 = (double)(b2 - a22 * x2) / (double)a21;
		return TRUE;
	}
	else
		return FALSE;
}

void GaussJordan3(double *A, double *B, double *X)
{
	double A2[9], B2[3];

	int i1 = 0;
	int i2 = 1;
	int i3 = 2;

	if(fabs(A[0 + 0 * 3]) < fabs(A[0 + 1 * 3]))
	{
	   i1 = 1;
	   i2 = 0;
	   i3 = 2;
	}
	   
	if(fabs(A[0 + i1 * 3]) < fabs(A[0 + 2 * 3]))
	{
	   i2 = i1;
	   i3 = 1 - i1;
	   i1 = 2;
	}
	
	double tmp1 = A[0 + i2 * 3] / A[0 + i1 * 3];
	A2[1 + i2 * 3] = A[1 + i2 * 3] - A[1 + i1 * 3] * tmp1;
	A2[2 + i2 * 3] = A[2 + i2 * 3] - A[2 + i1 * 3] * tmp1;
	B2[i2] = B[i2] - B[i1] * tmp1;

	tmp1 = A[0 + i3 * 3] / A[0 + i1 * 3];
	A2[1 + i3 * 3] = A[1 + i3 * 3] - A[1 + i1 * 3] * tmp1;
	A2[2 + i3 * 3] = A[2 + i3 * 3] - A[2 + i1 * 3] * tmp1;
	B2[i3] = B[i3] - B[i1] * tmp1;

	if(fabs(A2[1 + i2 * 3]) < fabs(A2[1 + i3 * 3]))
	{
		int tmp2 = i3;
		i3 = i2;
		i2 = tmp2;
	}

	tmp1 = A2[1 + i3 * 3] / A2[1 + i2 * 3];
	A2[2 + i3 * 3] = A2[2 + i3 * 3] - A2[2 + i2 * 3] * tmp1;
	B2[i3] = B2[i3] - B2[i2] * tmp1;

	X[2] = B2[i3] / A2[2 + i3 * 3];
	X[1] = (B2[i2] - A2[2 + i2 * 3] * X[2]) / A2[1 + i2 * 3];
	X[0] = (B[i1] - A[1 + i1 * 3] * X[1] - A[2 + i1 * 3] * X[2]) / A[0 + i1 * 3];
}

void MatrixMultiplication(double *A, double *B, double *Z, int n, int m)
{
	int i, j, k;
	double z;

	for(i = 0; i < n; i++)
		for(j = 0; j < m; j++)
		{
			z = 0.0;

			for(k = 0; k < n; k++)
				z += A[i * m + k] * B[k * m + j];

			Z[i * m + j] = z;
		}
}

// Z(n1 x n3) = A(n1 x n2) * B(n2 * n3)

void MatrixMultiplication(double *A, double *B, double *Z, int n1, int n2, int n3)
{
	int i, j, k;
	double z;

	for(i = 0; i < n1; i++)
		for(j = 0; j < n3; j++)
		{
			z = 0.0;

			for(k = 0; k < n2; k++)
				z += A[i * n2 + k] * B[k * n3 + j];

			Z[i * n3 + j] = z;
		}
}

// Z(n1 x n3) = A(n1 x n2) * B'(n3 * n2)

void MatrixMultiplicationT(double *A, double *B, double *Z, int n1, int n2, int n3)
{
	int i, j, k;
	double z;

	for(i = 0; i < n1; i++)
		for(j = 0; j < n3; j++)
		{
			z = 0.0;

			for(k = 0; k < n2; k++)
				z += A[i * n2 + k] * B[j * n2 + k];

			Z[i * n3 + j] = z;
		}
}

void MatrixMultiplicationT(double *A, double *B, double *Z, int n, int m)
{
	int i, j, k;
	double z;

	for(i = 0; i < n; i++)
		for(j = 0; j < m; j++)
		{
			z = 0.0;

			for(k = 0; k < n; k++)
				z += A[i * m + k] * B[j * m + k];

			Z[i * m + j] = z;
		}
}

// Z(n1 x n3) = A'(n2 x n1) * B(n2 * n3)

void MatrixMultiplicationT2(double *A, double *B, double *Z, int n1, int n2, int n3)
{
	int i, j, k;
	double z;

	for(i = 0; i < n1; i++)
		for(j = 0; j < n3; j++)
		{
			z = 0.0;

			for(k = 0; k < n2; k++)
				z += A[k * n1 + i] * B[k * n3 + j];

			Z[i * n3 + j] = z;
		}
}


int *RVLGetsqrt_LookUpTable()
{
	return sqrt_LookUpTable;
};

int rtsqrt(int x)
{
	if(x > 65280)
	{
		int n = 0;
		int x2 = x;

		while(x2 > 65280)
		{
			x2 = (x2 >> 2);
			n++;
		}

		return ((sqrt_LookUpTable[x2] + 1) << n) - 1;
	}

	return sqrt_LookUpTable[x];
}

void rtUtilInit()
{
	int i;

	for(i = 0; i <= 65280; i++)
		sqrt_LookUpTable[i] = DOUBLE2INT(sqrt((double)i));

	for(i = 0; i <= 1000; i++)
		atan45_LookUpTable[i] = (BYTE)DOUBLE2INT(atan(((double)i) * 0.001) * RAD2NRM * 2);

	for(i = 5; i <= 2000; i++)
		log10_LookUpTable[i] = (BYTE)floor(log10(((double)i) * 0.01) * 97.0 + 128.5);

	FILE *fp;

	fp = fopen("C:\\RVL\\chi2cdf3D.dat", "r");

	if(fp)
	{
		double x;
		double P;

		for(i = 0; i <= 1200; i++)
		{
			fscanf(fp, "%lf %lf\n", &x, &P);

			chi2_LookUpTable[i] = DOUBLE2INT(100.0 * P);
		}

		fclose(fp);
	}

	lnFactorial_LookUpTable[0] = 1;

	double lnFactorial = 0.0;

	for(i = 1; i <= 100; i++)
	{
		lnFactorial += log((double)i);

		lnFactorial_LookUpTable[i] = lnFactorial;
	}
}

// Real-time angle calculation. [0 to 90 deg] -> [0 to 120]

BYTE rtAngle90(int dx, int dy)
{
	int absdx = abs(dx);
	int absdy = abs(dy);

	if(absdy > absdx)
		return 120 - atan45_LookUpTable[absdx * 1000 / absdy];
	else
		return atan45_LookUpTable[absdy * 1000 / absdx];
}

// Real-time angle calculation. [0 to 180 deg] -> [0 to 240]

BYTE rtAngle180(int dx, int dy)
{
	int absdx = abs(dx);
	int absdy = abs(dy);
	BYTE AngleCode;

	if(absdx > absdy)
		AngleCode = atan45_LookUpTable[absdy * 1000 / absdx];
	else
		AngleCode = 120 - atan45_LookUpTable[absdx * 1000 / absdy];

	if(dy < 0)
		dx = -dx;

	if(dx < 0)
		AngleCode = 240 - AngleCode;

	return AngleCode;
}

// Real-time angle calculation. [-90 to 90 deg] -> [0 to 240]

BYTE rtAngle180_2(int dx, int dy)
{
	int absdx = abs(dx);
	int absdy = abs(dy);
	BYTE AngleCode;

	if(absdx > absdy)
		AngleCode = atan45_LookUpTable[absdy * 1000 / absdx];
	else
		AngleCode = 120 - atan45_LookUpTable[absdx * 1000 / absdy];

	if(dx < 0)
		dy =-dy;

	if(dy >= 0)
		AngleCode += 120;
	else
		AngleCode = 120 - AngleCode;

	return AngleCode;
}

// Real-time angle calculation. [-180 to 180 deg] -> [0 to 240]

BYTE rtAngle360(int dx, int dy)
{
	int absdx = abs(dx);
	int absdy = abs(dy);
	BYTE AngleCode;

	if(absdx > absdy)
		AngleCode = (atan45_LookUpTable[absdy * 1000 / absdx] >> 1);
	else
		AngleCode = 60 - (atan45_LookUpTable[absdx * 1000 / absdy] >> 1);

	if(dx >= 0)
	{
		if(dy >= 0)
			AngleCode += 120;
		else
			AngleCode = 120 - AngleCode;
	}
	else
	{
		if(dy >= 0)
			AngleCode = 240 - AngleCode;
	}

	return AngleCode;
}


BYTE rtlog10(int x)
{
	return log10_LookUpTable[x];
}

BYTE *CreateLogLookUpTable(int MinX, int MaxX, int CellSize, double Base)
{
	BYTE *LogLookUpTable = new BYTE[MaxX + 1];

	int i;

	for(i = 0; i < MinX; i++)
		LogLookUpTable[i] = 0;

	double Min = (double)MinX;
	double coeff1 = (double)CellSize / log(Base);

	for(i = MinX; i <= MaxX; i++)
		LogLookUpTable[i] = (BYTE)floor(log((double)i / MinX) * coeff1 + 0.5);

	return LogLookUpTable;
}

void PanTiltRoll(double PanAngle, double TiltAngle, double RollAngle, double *Rotation/*, double *InvRotation*/)
{
	double alpha = PanAngle * DEG2RAD;
	double beta = TiltAngle * DEG2RAD;
	double theta = RollAngle * DEG2RAD;
	
	double ca = cos(alpha);
	double sa = sin(alpha);
	double cb = cos(beta);
	double sb = sin(beta);
	double cq = cos(theta);
	double sq = sin(theta);

	/*
	
			[  ca,   0,  sa]
	Rota =	[   0,   1,   0]
			[ -sa,   0,  ca]

			[   1,   0,   0]
	Rotb = 	[   0,  cb, -sb]
			[   0,  sb,  cb]

			[  cq, -sq,   0]
	Rotq =	[  sq,  cq,   0]
			[   0,   0,   1]
			
	Rotation = Rota * Rotb * Rotq

	*/
	
	Rotation[0 + 0 * 3] = ca*cq+sa*sb*sq;
	Rotation[1 + 0 * 3] = -ca*sq+sa*sb*cq;
	Rotation[2 + 0 * 3] = sa*cb;
	Rotation[0 + 1 * 3] = cb*sq;
	Rotation[1 + 1 * 3] = cb*cq;
	Rotation[2 + 1 * 3] = -sb;
	Rotation[0 + 2 * 3] = -sa*cq+ca*sb*sq;
	Rotation[1 + 2 * 3] = sa*sq+ca*sb*cq;
	Rotation[2 + 2 * 3] = ca*cb;
}

void PanTiltRoll(double *Rot, double &PanAngle, double &TiltAngle, double &RollAngle)
{
  PanAngle = atan(Rot[0 * 3 + 2] / Rot[2 * 3 + 2]) * RAD2DEG;
  TiltAngle = -asin(Rot[1 * 3 + 2]) * RAD2DEG;
  RollAngle = atan(Rot[1 * 3 + 0] / Rot[1 * 3 + 1]) * RAD2DEG;
}

void PTR2RPT(double PanPTR, double TiltPTR, double RollPTR,
	     double &PanRPT, double &TiltRPT, double &RollRPT)
{
	double alpha = PanPTR * DEG2RAD;
	double beta = TiltPTR * DEG2RAD;
	double theta = RollPTR * DEG2RAD;
	
	double ca = cos(alpha);
	double sa = sin(alpha);
	double cb = cos(beta);
	double sb = sin(beta);
	double cq = cos(theta);
	double sq = sin(theta);

	/*
	
			[  ca,   0,  sa]
	Rota =	[   0,   1,   0]
			[ -sa,   0,  ca]

			[   1,   0,   0]
	Rotb = 	[   0,  cb, -sb]
			[   0,  sb,  cb]

			[  cq, -sq,   0]
	Rotq =	[  sq,  cq,   0]
			[   0,   0,   1]
			
	RotPTR = Rota * Rotb * Rotq

	RotRPT = Rotq * Rota * Rotb

	*/
	
	PanRPT  = asin(sa*cq-ca*sb*sq) * RAD2DEG;
	TiltRPT = atan((sa*sq+ca*sb*cq)/(ca*cb)) * RAD2DEG;
	RollRPT = atan((cb*sq)/(ca*cq+sa*sb*sq)) * RAD2DEG;
}

void RPT2PTR(double PanRPT, double TiltRPT, double RollRPT,
	     double &PanPTR, double &TiltPTR, double &RollPTR)
{
	double alpha = PanRPT * DEG2RAD;
	double beta = TiltRPT * DEG2RAD;
	double theta = RollRPT * DEG2RAD;
	
	double ca = cos(alpha);
	double sa = sin(alpha);
	double cb = cos(beta);
	double sb = sin(beta);
	double cq = cos(theta);
	double sq = sin(theta);

	/*
	
			[  ca,   0,  sa]
	Rota =	[   0,   1,   0]
			[ -sa,   0,  ca]

			[   1,   0,   0]
	Rotb = 	[   0,  cb, -sb]
			[   0,  sb,  cb]

			[  cq, -sq,   0]
	Rotq =	[  sq,  cq,   0]
			[   0,   0,   1]
			
	RotPTR = Rota * Rotb * Rotq

	RotRPT = Rotq * Rota * Rotb

	*/
	


  PanPTR = atan((sq*sb+cq*sa*cb)/(ca*cb)) * RAD2DEG;
  TiltPTR = -asin(-cq*sb+sq*sa*cb) * RAD2DEG;
  RollPTR = atan((sq*ca)/(cq*cb+sq*sa*sb)) * RAD2DEG;

}

void Get2DRot(double *Rot, double Alpha)
{
	double alpha = Alpha * DEG2RAD;
	double cs = cos(alpha);
	double sn = sin(alpha);

	Rot[0] = cs;
	Rot[1] = 0.0;
	Rot[2] = -sn;
	Rot[3] = 0.0;
	Rot[4] = 1.0;
	Rot[5] = 0.0;
	Rot[6] = sn;
	Rot[7] = 0.0;
	Rot[8] = cs;
}


void Rotxz(double cs, double sn, double *X, double *Y)
{
	Y[0] = cs * X[0] + sn * X[2];
	Y[1] = X[1];
	Y[2] = -sn * X[0] + cs * X[2];
}

void InverseRotation(double *InvRotation, double *Rotation)
{
	InvRotation[0 + 0 * 3] = Rotation[0 + 0 * 3];
	InvRotation[1 + 0 * 3] = Rotation[0 + 1 * 3];
	InvRotation[2 + 0 * 3] = Rotation[0 + 2 * 3];
	InvRotation[0 + 1 * 3] = Rotation[1 + 0 * 3];
	InvRotation[1 + 1 * 3] = Rotation[1 + 1 * 3];
	InvRotation[2 + 1 * 3] = Rotation[1 + 2 * 3];
	InvRotation[0 + 2 * 3] = Rotation[2 + 0 * 3];
	InvRotation[1 + 2 * 3] = Rotation[2 + 1 * 3];
	InvRotation[2 + 2 * 3] = Rotation[2 + 2 * 3];
}
	
void RotFrom3DPoints(double *Rot, int *P11, int *P12, int *P21, int *P22)
{

/*

e11=p11+p12;
e11=e11/sqrt(e11'*e11);
m=p12-p11;
e12=m - (m'*e11)*e11;
e12=e12/sqrt(e12'*e12);
e13=cross(e11,e12);
R1=[e11 e12 e13];

e21=p21+p22;
e21=e21/sqrt(e21'*e21);
m=p22-p21;
e22=m - (m'*e21)*e21;
e22=e22/sqrt(e22'*e22);
e23=cross(e21,e22);
R2=[e21, e22, e23];

R=R2*R1';

alpha=atan(-R(3,1)/R(3,3))*180/pi;
beta=asin(R(3,2))*180/pi;
theta=atan(-R(1,2)/R(2,2))*180/pi;

*/

	// Rotation 0->1

	double x11 = (double)(P11[0] + P12[0]);
	double y11 = (double)(P11[1] + P12[1]);
	double z11 = (double)(P11[2] + P12[2]);
	double l = sqrt(x11 * x11 + y11 * y11 + z11 * z11);

	x11 = x11 / l;
	y11 = y11 / l;
	z11 = z11 / l;
	
	double xm = (double)(P12[0] - P11[0]);
	double ym = (double)(P12[1] - P11[1]);
	double zm = (double)(P12[2] - P11[2]);
	
	double a = xm * x11 + ym * y11 + zm * z11;
	double x12 = xm - a * x11;
	double y12 = ym - a * y11;
	double z12 = zm - a * z11;
	l = sqrt(x12 * x12 + y12 * y12 + z12 * z12);
	
	x12 = x12 / l;
	y12 = y12 / l;
	z12 = z12 / l;

	double x13 = y11 * z12 - z11 * y12;
	double y13 = z11 * x12 - x11 * z12;
	double z13 = x11 * y12 - y11 * x12;

	// Rotation 0->2

	double x21 = (double)(P21[0] + P22[0]);
	double y21 = (double)(P21[1] + P22[1]);
	double z21 = (double)(P21[2] + P22[2]);
	l = sqrt(x21 * x21 + y21 * y21 + z21 * z21);

	x21 = x21 / l;
	y21 = y21 / l;
	z21 = z21 / l;
	
	xm = (double)(P22[0] - P21[0]);
	ym = (double)(P22[1] - P21[1]);
	zm = (double)(P22[2] - P21[2]);
	
	a = xm * x21 + ym * y21 + zm * z21;
	double x22 = xm - a * x21;
	double y22 = ym - a * y21;
	double z22 = zm - a * z21;
	l = sqrt(x22 * x22 + y22 * y22 + z22 * z22);
	
	x22 = x22 / l;
	y22 = y22 / l;
	z22 = z22 / l;

	double x23 = y21 * z22 - z21 * y22;
	double y23 = z21 * x22 - x21 * z22;
	double z23 = x21 * y22 - y21 * x22;

	// Rotation 1->2

	Rot[0 * 3 + 0] = x21 * x11 + x22 * x12 + x23 * x13;
	Rot[0 * 3 + 1] = x21 * y11 + x22 * y12 + x23 * y13;
	Rot[0 * 3 + 2] = x21 * z11 + x22 * z12 + x23 * z13;
	Rot[1 * 3 + 0] = y21 * x11 + y22 * x12 + y23 * x13;
	Rot[1 * 3 + 1] = y21 * y11 + y22 * y12 + y23 * y13;
	Rot[1 * 3 + 2] = y21 * z11 + y22 * z12 + y23 * z13;
	Rot[2 * 3 + 0] = z21 * x11 + z22 * x12 + z23 * x13;
	Rot[2 * 3 + 1] = z21 * y11 + z22 * y12 + z23 * y13;
	Rot[2 * 3 + 2] = z21 * z11 + z22 * z12 + z23 * z13;
}
	
void PTRFrom3DPoints(double &Pan, double &Tilt, double &Roll, double &r22, double &r33, int *P11, int *P12, int *P21, int *P22)
{
	// Rotation 0->1

	double x11 = (double)(P11[0] + P12[0]);
	double y11 = (double)(P11[1] + P12[1]);
	double z11 = (double)(P11[2] + P12[2]);
	double l = sqrt(x11 * x11 + y11 * y11 + z11 * z11);

	x11 = x11 / l;
	y11 = y11 / l;
	z11 = z11 / l;
	
	double xm = (double)(P12[0] - P11[0]);
	double ym = (double)(P12[1] - P11[1]);
	double zm = (double)(P12[2] - P11[2]);
	
	double a = xm * x11 + ym * y11 + zm * z11;
	double x12 = xm - a * x11;
	double y12 = ym - a * y11;
	double z12 = zm - a * z11;
	l = sqrt(x12 * x12 + y12 * y12 + z12 * z12);
	
	x12 = x12 / l;
	y12 = y12 / l;
	z12 = z12 / l;

	double x13 = y11 * z12 - z11 * y12;
	double y13 = z11 * x12 - x11 * z12;
	double z13 = x11 * y12 - y11 * x12;

	// Rotation 0->2

	double x21 = (double)(P21[0] + P22[0]);
	double y21 = (double)(P21[1] + P22[1]);
	double z21 = (double)(P21[2] + P22[2]);
	l = sqrt(x21 * x21 + y21 * y21 + z21 * z21);

	x21 = x21 / l;
	y21 = y21 / l;
	z21 = z21 / l;
	
	xm = (double)(P22[0] - P21[0]);
	ym = (double)(P22[1] - P21[1]);
	zm = (double)(P22[2] - P21[2]);
	
	a = xm * x21 + ym * y21 + zm * z21;
	double x22 = xm - a * x21;
	double y22 = ym - a * y21;
	double z22 = zm - a * z21;
	l = sqrt(x22 * x22 + y22 * y22 + z22 * z22);
	
	x22 = x22 / l;
	y22 = y22 / l;
	z22 = z22 / l;

	double x23 = y21 * z22 - z21 * y22;
	double y23 = z21 * x22 - x21 * z22;
	double z23 = x21 * y22 - y21 * x22;

	// Rotation 1->2

	double r12 = x21 * y11 + x22 * y12 + x23 * y13;
	r22 = y21 * y11 + y22 * y12 + y23 * y13;
	double r31 = z21 * x11 + z22 * x12 + z23 * x13;
	double r32 = z21 * y11 + z22 * y12 + z23 * y13;
	r33 = z21 * z11 + z22 * z12 + z23 * z13;

	Pan = atan(-r31 / r33) * RAD2DEG;
	Tilt = asin(r32) * RAD2DEG;
	Roll = atan(-r12 / r22) * RAD2DEG;
}

// T(Rot, Trans) = T(Rot1, Trans1) * T(Rot2, Trans2)
	
void RVLCombineTransform3D(double *Rot1, double *Trans1, double *Rot2, double *Trans2,
						double *Rot, double *Trans)
{
	MatrixMultiplication(Rot1, Rot2, Rot, 3, 3);
	LinearTransform3D(Rot1, Trans2, Trans);
	Trans[0] += Trans1[0];
	Trans[1] += Trans1[1];
	Trans[2] += Trans1[2];
}

void InverseTransform3D(double *InvRotation, double *InvTranslation, double *Rotation,
						double *Translation)
{
	InverseRotation(InvRotation, Rotation);

	LinearTransform3D(InvRotation, Translation, InvTranslation);

	InvTranslation[0] = -InvTranslation[0];
	InvTranslation[1] = -InvTranslation[1];
	InvTranslation[2] = -InvTranslation[2];
}

void InverseMatrix3(double *InvA, double *A)
{
	InverseMatrix3(InvA, A, 1e-15);
}

BOOL InverseMatrix3(double *InvA, double *A, double MinDet)
{
	int i;

	InvA[0 * 3 + 0] =   A[1 * 3 + 1] * A[2 * 3 + 2] - A[1 * 3 + 2] * A[2 * 3 + 1];
	InvA[1 * 3 + 0] = -(A[1 * 3 + 0] * A[2 * 3 + 2] - A[1 * 3 + 2] * A[2 * 3 + 0]);
	InvA[2 * 3 + 0] =   A[1 * 3 + 0] * A[2 * 3 + 1] - A[1 * 3 + 1] * A[2 * 3 + 0];
	InvA[0 * 3 + 1] = -(A[0 * 3 + 1] * A[2 * 3 + 2] - A[0 * 3 + 2] * A[2 * 3 + 1]);
	InvA[1 * 3 + 1] =   A[0 * 3 + 0] * A[2 * 3 + 2] - A[0 * 3 + 2] * A[2 * 3 + 0];
	InvA[2 * 3 + 1] = -(A[0 * 3 + 0] * A[2 * 3 + 1] - A[0 * 3 + 1] * A[2 * 3 + 0]);
	InvA[0 * 3 + 2] =   A[0 * 3 + 1] * A[1 * 3 + 2] - A[0 * 3 + 2] * A[1 * 3 + 1];
	InvA[1 * 3 + 2] = -(A[0 * 3 + 0] * A[1 * 3 + 2] - A[0 * 3 + 2] * A[1 * 3 + 0]);
	InvA[2 * 3 + 2] =   A[0 * 3 + 0] * A[1 * 3 + 1] - A[0 * 3 + 1] * A[1 * 3 + 0];

	double det = A[0 * 3 + 0] * InvA[0 * 3 + 0] +
			     A[0 * 3 + 1] * InvA[1 * 3 + 0] + 
				 A[0 * 3 + 2] * InvA[2 * 3 + 0];

	if(fabs(det) < MinDet)
		return FALSE;

	double *pInvA = InvA;

	for(i = 0; i < 9; i++)
	{
		*pInvA /= det;

		pInvA++;
	}

	return TRUE;
}

BOOL InverseMatrix2(double *InvA, double *A, double MinDet)
{
	double det = A[0] * A[3] - A[1] * A[2];

	if(fabs(det) < MinDet)
		return FALSE;

	double detrec = 1.0 / det;

	InvA[0] = A[3] * detrec;
	InvA[1] = -A[1] * detrec;
	InvA[2] = -A[2] * detrec;
	InvA[3] = A[0] * detrec;

	return TRUE;
}

void Sort(int *Key, int *Index, int n, BOOL bIndex)
{
  int i, j;
  int Min;
  int iMin;
  int tmp;

  if(!bIndex)
	for(i = 0; i < n; i++)
		Index[i] = i;

  for(i = 0; i < n; i++)
    {
      Min = Key[i];
      iMin = i;

      for(j = i + 1; j < n; j++)
	if(Key[j] < Min)
	  {
	    Min = Key[j];
	    iMin = j;
	  }
      
      Key[iMin] = Key[i];
      Key[i] = Min;
      tmp = Index[i];
      Index[i] = Index[iMin];
      Index[iMin] = tmp;
    }
}

void Sort(double *Key, int *Index, int n, BOOL bIndex)
{
  int i, j;
  double Min;
  int iMin;
  int tmp;

  if(!bIndex)
	for(i = 0; i < n; i++)
		Index[i] = i;

  for(i = 0; i < n; i++)
    {
      Min = Key[i];
      iMin = i;

      for(j = i + 1; j < n; j++)
	if(Key[j] < Min)
	  {
	    Min = Key[j];
	    iMin = j;
	  }
      
      Key[iMin] = Key[i];
      Key[i] = Min;
      tmp = Index[i];
      Index[i] = Index[iMin];
      Index[iMin] = tmp;
    }
}

void ANDDWArray(DWORD *X1, DWORD *X2, DWORD *Y, int n)
{
	int i;

	for(i = 0; i < n; i++)
		Y[i] = (X1[i] & X2[i]);
}

void ORDWArray(DWORD *X1, DWORD *X2, DWORD *Y, int n)
{
	int i;

	for(i = 0; i < n; i++)
		Y[i] = (X1[i] | X2[i]);
}

BOOL EqualDWArray(DWORD *X1, DWORD *X2, int n)
{
	int i;

	for(i = 0; i < n; i++)
		if(X1[i] != X2[i])
			return FALSE;

	return TRUE;
}

BOOL IsInAngle(int Angle, int AngleMin, int AngleMax)
{
	if(AngleMin < AngleMax)
		return ((Angle - AngleMin) * (Angle - AngleMax) < 0);
	else
		return ((Angle - AngleMin) * (Angle - AngleMax) > 0);
}

void CrossProduct(double *X1, double *X2, double *Y)
{
	Y[0] = X1[1] * X2[2] - X1[2] * X2[1];
	Y[1] = X1[2] * X2[0] - X1[0] * X2[2];
	Y[2] = X1[0] * X2[1] - X1[1] * X2[0];
}

// Z=A'*A, A(n x m)

void MultiplicationWithTransposed(double *A, double *Z, int n, int m)
{
	int i, j, k;
	double z;

	for(i = 0; i < m; i++)
		for(j = 0; j < m; j++)
		{
			z = 0.0;

			for(k = 0; k < n; k++)
				z += A[k * m + i] * A[k * m + j];

			Z[i * m + j] = z;
		}
}

//! Not functional !

BOOL GetMinEigenvector(double *M, double *q, double &d, int n)
{
	//double *D;
	//double **V, **M2;

	//! D = vector(1, n);
	//! V = matrix(1, n, 1, n);
	//! M2 = convert_matrix(M, 1, n, 1, n);

	//! int nrot;

	//! if(!jacobi(M2, n, D, V, &nrot))
	//!   return FALSE;

	//d = D[1];
	//int iMinD = 1;

	//int i;

	//for(i = 2; i <= n; i++)
	//	if(D[i] < d)
	//	{
	//		d = D[i];
	//		iMinD = i;
	//	}

	//for(i = 0; i < n; i++)
	//	q[i] = V[i + 1][iMinD];

	return TRUE;
}

void Robot2Camera(double *RotR, double *RotC)
{
  RotC[0 * 3 + 0] = RotR[1 * 3 + 1];
  RotC[0 * 3 + 1] = RotR[1 * 3 + 2];
  RotC[0 * 3 + 2] = RotR[1 * 3 + 0];
  RotC[1 * 3 + 0] = RotR[2 * 3 + 1];
  RotC[1 * 3 + 1] = RotR[2 * 3 + 2];
  RotC[1 * 3 + 2] = RotR[2 * 3 + 0];
  RotC[2 * 3 + 0] = RotR[0 * 3 + 1];
  RotC[2 * 3 + 1] = RotR[0 * 3 + 2];
  RotC[2 * 3 + 2] = RotR[0 * 3 + 0];
}

// returns real roots of a 2nd order polynomial

BOOL Roots2(double *p, double *z)
{
	double det = p[1] * p[1] - 4.0 * p[0] * p[2];

	if(det < 0.0)
		return FALSE;

	double fTmp1 = 2.0 * p[2];
	double fTmp2 = -p[1] / fTmp1;
	double fTmp3 = sqrt(det) / fTmp1;

	z[0] = fTmp2 - fTmp3;
	z[1] = fTmp2 + fTmp3;

	return TRUE;		
}

// returns real roots of a 3rd order polynomial

void Roots3(double *p, double *z, BOOL *bReal)
{
	double q0 = p[0] / p[3];
	double q1 = p[1] / p[3];
	double q2 = p[2] / p[3];
	double q2_2 = q2 * q2;
	double r = q1 / 3.0 - q2_2 / 9.0;
	double s = (q2_2 / 27.0 - q1 / 6.0) * q2 + 0.5 * q0;
	double D = s * s + r * r * r;
	double t, u, v;
	double fTmp;

	if(D > 1e-20)
	{
		t = sqrt(D);
		fTmp = -s + t;
		u = pow(fabs(fTmp), 1.0 / 3.0);
		if(fTmp < 0)
			u = -u;
		fTmp = -s - t;
		v = pow(fabs(fTmp), 1.0 / 3.0);		
		if(fTmp < 0)
			v = -v;
		z[0] = u + v - q2 / 3.0;
		bReal[0] = TRUE;
		bReal[1] = bReal[2] = FALSE;
	}
	else if(D < -1e-20)
	{
		double w = sqrt(fabs(r));
		double cp = s / (w * w * w);
		//z[0] = - 2.0 * w * cp - q2 / 3.0;
		//double phi = acos(w);
		double phi = acos(cp);
		z[0] = -2.0 * w * cos(phi / 3.0) - q2 / 3.0;
		/////
		z[1] = 2.0 * w * cos((PI - phi) / 3.0) - q2 / 3.0;
		z[2] = 2.0 * w * cos((PI + phi) / 3.0) - q2 / 3.0;
		bReal[0] = bReal[1] = bReal[2] = TRUE;
	}
	else
	{
		z[0] = z[1] = z[2] = -q2 / 3.0;

		bReal[0] = bReal[1] = bReal[2] = TRUE;
	}
}

// returns real roots of a 4rd order polynomial

void Roots4(double *p, double *z, BOOL *bReal)
{
	double b = p[3] / p[4];
	double c = p[2] / p[4];
	double d = p[1] / p[4];
	double e = p[0] / p[4];

	double y2[3], r[4];
	BOOL bReal2[3];

	double c4 = 4.0 * c;
	double b_2 = b * b;

	r[0] = e * (c4 - b_2) - d * d;
	r[1] = (2.0 * b * d - 8.0 * e);
	r[2] = -c4;
	r[3] = 8.0;

	Roots3(r, y2, bReal2);

	double y = y2[0];

	// For debugging purpose only!

	//double fTmp = ((r[3] * y + r[2]) * y + r[1]) * y + r[0];

	/////

	double D = 8.0 * y + b_2 - c4;

	int i;

	if(D > 0.0)
	{
		double A = sqrt(D);

		double p, q, det;

		for(i = 0; i < 2; i++)
		{
			p = (b + A) / 2.0;
			q = y + (b * y - d) / A;
			det = p * p - 4.0 * q;

			if(det >= 0.0)
			{
				double s = sqrt(det);
				z[2 * i + 0] = (-p + s) * 0.5;
				z[2 * i + 1] = (-p - s) * 0.5;
				bReal[2 * i + 0] = bReal[2 * i + 1] = TRUE;
			}
			else
				bReal[2 * i + 0] = bReal[2 * i + 1] = FALSE;

			A = -A;
		}
	}
	else 
		for(i = 0; i < 4; i++)
			bReal[i] = FALSE;	
}

// given a point [x y]' in c.s. SA and pose of c.s. SB relative to SA defined by
// orientation (cs, sn) and position [x0 y0]', 
// calculates the coordinates [p q] of the point in SB:
//
// [p q]' = R' * ([x y]' - [x0 y0]'), R = [cs -sn; sn cs];

void LinearTransform2D(double cs, double sn, double x0, double y0, double x, double y, 
					   double &p, double &q)
{
	double x1 = x - x0;
	double y1 = y - y0;

	p = cs * x1 + sn * y1;
	q = -sn * x1 + cs * y1;
}

// Y(3 x 1) = A'(3 x 3) * (X(3 x 1) - b(3 x 1))

void InvLinearTransform3D(double *A, double *b, double *X, double *Y)
{
	double X1[3];

	RVLDif3D(X, b, X1);

	LinearTransform3DT(A, X1, Y);
}

int Point2Line2DDist(int dx, int dy, int len, int x0, int y0, int x, int y)
{
	int x1 = x - x0;
	int y1 = y - y0;

	return (-dy * x1 + dx * y1) / len;
}


// given a point [x y]' in c.s. SA and pose of c.s. SB relative to SA 
// defined by orientation (cs, sn) and position [x0 y0]', 
// calculates the coordinates [p q] of the point in SB:
//
// [p q]' = R' * ([x y]' - [x0 y0]'), R = [cs -sn; sn cs]

void LinearTransform2D(int dx, int dy, int len, int x0, int y0, int x, int y, int &p, int &q)
{
	int x1 = x - x0;
	int y1 = y - y0;

	p = (dx * x1 + dy * y1) / len;
	q = (-dy * x1 + dx * y1) / len;
}

// given a point [x y]' in c.s. SA and pose of SA relative to a c.s. SB 
// defined by orientation (cs, sn) and position [x0 y0]', 
// calculates the coordinates [p q] of the point in SB:
//
// [p q]' = R * [x y]' + [x0 y0]'

void LinearTransform2D2(double cs, double sn, double x0, double y0, double x, double y, 
					   double &p, double &q)
{
	p = cs * x - sn * y + x0;
	q = sn * x + cs * y + y0;
}

// given ellipse c11*x^2+2*c12*x*y+c22*y^2=1, calculate radii R1 and R2 and angle Phi of ellipse,
// where R1 >= R2

void GetEllipseParams(double c11, double c12, double c22, double &R1, double &R2, double &phi)
{
	double k1 = c11 + c22;
	double k2 = c22 - c11;
	double k3 = 2 * c12;
	double k4 = sqrt(k2 * k2 + k3 * k3);
	
	phi = atan2(-k3, k2) * 0.5;

	R1 = sqrt(2.0 / (k1 - k4));
	R2 = sqrt(2.0 / (k1 + k4));
}

// given ellipse c11*x^2+2*c12*x*y+c22*y^2=1, calculate the main axis of the ellipse

BOOL GetEllipseAxis(double c11, double c12, double c22, double &p1, double &p2, double &tgPhi)
{
	double dc = c11 - c22;

	double c122 = 2.0 * c12;

	if(fabs(c12) < APPROX_ZERO)
		return FALSE;

	double k = dc / c122; 

	tgPhi = -k + (c12 > 0.0 ? -sqrt(k * k + 1.0) : sqrt(k * k + 1.0));

	double tgPhi2 = tgPhi * tgPhi;

	double fTmp2 = c122 * tgPhi;

	p1 = sqrt(1.0 / (c11 + fTmp2 + c22 * tgPhi2));
	p2 = p1 * tgPhi;

	return TRUE;
}

void RVLQuickSort(short *Key, int *Index, int n)
{
	// get range

	short *pKey = Key;

	int min = *(pKey++);
	int max = min;

	int i;

	for(i = 1; i < n; i++, pKey++)
	{
		if(*pKey < min)
			min = *pKey;
		else if(*pKey > max)
			max = *pKey;
	}

	int nBins = max - min + 1;

	// create lookup table

	CRVLWChain **KeyLT = new CRVLWChain *[nBins];

	memset(KeyLT, 0, nBins * sizeof(CRVLWChain *));

	pKey = Key;

	CRVLWChain **ppKeyLT;

	for(i = 0; i < n; i++, pKey++)
	{
		ppKeyLT = KeyLT + *pKey - min;

		if(*ppKeyLT == NULL)
			*ppKeyLT = new CRVLWChain;

		(*ppKeyLT)->Add((WORD)i);
	}

	// fill index array

	int *pIndex = Index;

	ppKeyLT = KeyLT;

	for(i = 0; i < nBins; i++, ppKeyLT++)
		if(*ppKeyLT != NULL)
		{
			(*ppKeyLT)->Start();

			while((*ppKeyLT)->m_pNext)
				*(pIndex++) = (int)((*ppKeyLT)->GetNext());

			delete *ppKeyLT;
		}

	// deallocate lookup table

	delete[] KeyLT;
}

BOOL RVLEig2(double *C, double *eig)
{
	double p[3];

	p[2] = 1.0;
	p[1] = -(C[0] + C[3]);
	p[0] = C[0] * C[3] - C[1] * C[2];

	return Roots2(p, eig);
}

BOOL RVLGetMaxEigVector2(double *C, double *eig, double *Veig)
{
	if(!RVLEig2(C, eig))
		return FALSE;
	
	double maxeig = (eig[0] > eig[1] ? eig[0] : eig[1]);

	Veig[0] = (maxeig - C[3]) / C[2];

	Veig[1] = 1.0 / sqrt(Veig[0] * Veig[0] + 1.0);

	Veig[0] *= Veig[1];

	return TRUE;
}

void RVLEig3(double *C, double *eig, BOOL *bReal)
{
	double p[4];

	p[3] = 1.0;
	p[2] = -C[0 * 3 + 0]-C[2 * 3 + 2]-C[1 * 3 + 1];
	p[1] = -C[2 * 3 + 0]*C[0 * 3 + 2]+C[0 * 3 + 0]*C[2 * 3 + 2]+C[1 * 3 + 1]*C[2 * 3 + 2]-
		C[1 * 3 + 0]*C[0 * 3 + 1]+C[0 * 3 + 0]*C[1 * 3 + 1]-C[1 * 3 + 2]*C[2 * 3 + 1];
	p[0] = -C[2 * 3 + 0]*C[0 * 3 + 1]*C[1 * 3 + 2]+C[0 * 3 + 0]*C[1 * 3 + 2]*C[2 * 3 + 1]+
		C[2 * 3 + 0]*C[0 * 3 + 2]*C[1 * 3 + 1]+C[1 * 3 + 0]*C[0 * 3 + 1]*C[2 * 3 + 2]-
		C[0 * 3 + 0]*C[1 * 3 + 1]*C[2 * 3 + 2]-C[1 * 3 + 0]*C[0 * 3 + 2]*C[2 * 3 + 1];

	Roots3(p, eig, bReal);
}

BOOL RVLGetMinEigVector3(double *C, double *eig, BOOL *bReal, double *Veig)
{
	RVLEig3(C, eig, bReal);
	
	if(!bReal[0] || !bReal[1] || !bReal[2])
		return FALSE;

	Sort3(eig);

	TwoEqsTwoVars(eig[0] - C[0 * 3 + 0], -C[0 * 3 + 1],  -C[1 * 3 + 0], eig[0] - C[1 * 3 + 1],
		C[0 * 3 + 2], C[1 * 3 + 2], Veig[0], Veig[1]);

	Veig[2] = 1.0 / sqrt(Veig[0] * Veig[0] + Veig[1] * Veig[1] + 1.0);

	Veig[0] *= Veig[2];
	Veig[1] *= Veig[2];

	return TRUE;
}

BOOL RVLGetMaxEigVector3(double *C, double *eig, BOOL *bReal, double *Veig)
{
	RVLEig3(C, eig, bReal);
	
	if(!bReal[0] || !bReal[1] || !bReal[2])
		return FALSE;

	Sort3(eig);

	TwoEqsTwoVars(eig[2] - C[0 * 3 + 0], -C[0 * 3 + 1],  -C[1 * 3 + 0], eig[2] - C[1 * 3 + 1],
		C[0 * 3 + 2], C[1 * 3 + 2], Veig[0], Veig[1]);

	Veig[2] = 1.0 / sqrt(Veig[0] * Veig[0] + Veig[1] * Veig[1] + 1.0);

	Veig[0] *= Veig[2];
	Veig[1] *= Veig[2];

	return TRUE;
}

void RVLGetCovMatrix2(RVL2DMOMENTS *pMoments, double *C, double *M)
{
	double fn = (double)(pMoments->n);
	M[0] = pMoments->S[0] / fn;
	M[1] = pMoments->S[1] / fn;
	C[0] = pMoments->S2[0] / fn - M[0] * M[0];
	C[1] = pMoments->S2[1] / fn - M[0] * M[1];
	C[2] = C[1];
	C[3] = pMoments->S2[3] / fn - M[1] * M[1];
}

void RVLGetCovMatrix3(RVL3DMOMENTS *pMoments, double *C, double *M)
{
	double fn = (double)(pMoments->n);
	M[0] = pMoments->S[0] / fn;
	M[1] = pMoments->S[1] / fn;
	M[2] = pMoments->S[2] / fn;
	C[0] = pMoments->S2[0] / fn - M[0] * M[0];
	C[1] = pMoments->S2[1] / fn - M[0] * M[1];
	C[2] = pMoments->S2[2] / fn - M[0] * M[2];
	C[3] = C[1];
	C[4] = pMoments->S2[4] / fn - M[1] * M[1];
	C[5] = pMoments->S2[5] / fn - M[1] * M[2];
	C[6] = C[2];
	C[7] = C[5];
	C[8] = pMoments->S2[8] / fn - M[2] * M[2];
}

void RVLHSB2RGB(int h, int s, int k, int &r, int &g, int &b)
{
	switch(h / 256){
	case 0:
		r = 255;
		g = 255 - s;
		b = 255 - h % 256;
		break;
	case 1:
		r = 255;
		g = h % 256;
		b = 255 - s;
		break;
	case 2:
		r = 255 - h % 256;
		g = 255;
		b = 255 - s;
		break;
	case 3:
		r = 255 - s;
		g = 255;
		b = h % 256;
		break;
	case 4:
		r = 255 - s;
		g = 255 - h % 256;
		b = 255;
		break;
	case 5:
		r = h % 256;
		g = 255 - s;
		b = 255;
	}
	
	int l = 255 - k;
	r = r * l / 255;
	g = g * l / 255;
	b = b * l / 255;
}

void CreateUnitMatrix(double *A, int n)
{
	memset(A, 0, n * n * sizeof(double));

	double *pA = A;

	int i;

	for(i = 0; i < n; i++, pA += (n + 1))
		*pA = 1.0;
}

void Sort3(double *X)
{
	double fTmp;

	if(X[1] < X[0])
	{
		fTmp = X[0];
		X[0] = X[1];
		X[1] = fTmp;
	}

	if(X[2] < X[0])
	{
		fTmp = X[0];
		X[0] = X[2];
		X[2] = fTmp;
	}

	if(X[2] < X[1])
	{
		fTmp = X[1];
		X[1] = X[2];
		X[2] = fTmp;
	}
}

// rotation of 2D covariance matrix CRot = Rot * C * Rot'; Rot = [cs -sn; sn cs]
// (C is simetric)

void RotCov2D(double *C, double cs, double sn, double *CRot)
{
	double cs2 = cs * cs;
	double cssn = cs * sn;
	double cssn2 = 2.0 * cssn;
	double sn2 = sn * sn;

	CRot[0] = cs2 * C[0] - cssn2 * C[1] + sn2 * C[3];
	CRot[1] = CRot[2] = -sn2 * C[1] - (C[3] - C[0]) * cssn + cs2 * C[1];
	CRot[3] = sn2 * C[0] + cssn2 * C[1] + cs2 * C[3];
}

// rotation of 3D covariance matrix CRot = Rot * C * Rot'

void RotCov3D(double *C, double *Rot, double *CRot)
{
	double CTmp[9];

	MatrixMultiplication(Rot, C, CTmp, 3, 3, 3);

	MatrixMultiplicationT(CTmp, Rot, CRot, 3, 3, 3);
}

void RVLUnitMatrix(double *M, int n)
{
	memset(M, 0, n * n * sizeof(double));

	double *pM = M;

	int i;

	for(i = 0; i < n; i++, pM += (n + 1))
		*pM = 1.0;
}

void RVLGetMainAxisOfCov3D(double *C, double *W3D)
{
	double eig[3];
	BOOL bReal[3];

	RVLGetMaxEigVector3(C, eig, bReal, W3D);

	double lW = sqrt(eig[2]);

	W3D[0] *= lW;
	W3D[1] *= lW;
	W3D[2] *= lW;
}

BOOL RVLGetAxesOfCov3D(double *C, double *eig, double *VNrm, double *V, double *lV)
{
	BOOL bReal[3];	

	RVLEig3(C, eig, bReal);
	
	if(!bReal[0] || !bReal[1] || !bReal[2])
		return FALSE;

	Sort3(eig);

	double *pV = V;		
	double *pVNrm = VNrm;
	double *plV = lV;
	double *peig = eig;

	WORD kAxis;

	for(kAxis = 0; kAxis < 3; kAxis++, peig++, pV += 3, pVNrm += 3, plV++)
	{
		TwoEqsTwoVars(*peig - C[0 * 3 + 0], -C[0 * 3 + 1],  -C[1 * 3 + 0], *peig - C[1 * 3 + 1],
			C[0 * 3 + 2], C[1 * 3 + 2], pVNrm[0], pVNrm[1]);

		pVNrm[2] = 1.0 / sqrt(pVNrm[0] * pVNrm[0] + pVNrm[1] * pVNrm[1] + 1.0);

		pVNrm[0] *= pVNrm[2];
		pVNrm[1] *= pVNrm[2];

		*plV = sqrt(*peig);

		pV[0] = pVNrm[0] * (*plV);
		pV[1] = pVNrm[1] * (*plV);
		pV[2] = pVNrm[2] * (*plV);
	}

	return TRUE;
}

BOOL RVLGetEigVects3(double *C, 
				     double *eig, 
				     double *V)
{
	BOOL bReal[3];	

	RVLEig3(C, eig, bReal);
	
	if(!bReal[0] || !bReal[1] || !bReal[2])
		return FALSE;

	double w1, w2, w3;
	double *pV = V;		
	double *peig = eig;

	WORD kAxis;

	for(kAxis = 0; kAxis < 3; kAxis++, peig++, pV++)
	{
		TwoEqsTwoVars(*peig - C[0 * 3 + 0], -C[0 * 3 + 1],  -C[1 * 3 + 0], *peig - C[1 * 3 + 1],
			C[0 * 3 + 2], C[1 * 3 + 2], w1, w2);

		w3 = 1.0 / sqrt(w1 * w1 + w2 * w2 + 1.0);

		*(pV + 3*0) = w1 * w3;
		*(pV + 3*1) = w2 * w3;
		*(pV + 3*2) = w3;
	}

	return TRUE;
}

double RVLMahalanobisDistance2D(double *dX, double *C2D)
{
	double InvC2D[4];

	InverseMatrix2(InvC2D, C2D, APPROX_ZERO);

	return InvC2D[0] * dX[0] * dX[0] + (InvC2D[1] + InvC2D[2]) * dX[0] * dX[1] + InvC2D[3] * dX[1] * dX[1];	
}

void RVLMomentsSum(RVL3DMOMENTS *pMoments1, RVL3DMOMENTS *pMoments2, RVL3DMOMENTS *pMomentsTgt)
{
	pMomentsTgt->n = pMoments1->n + pMoments2->n;
	pMomentsTgt->S[0] = pMoments1->S[0] + pMoments2->S[0];
	pMomentsTgt->S[1] = pMoments1->S[1] + pMoments2->S[1];
	pMomentsTgt->S[2] = pMoments1->S[2] + pMoments2->S[2];
	pMomentsTgt->S2[0] = pMoments1->S2[0] + pMoments2->S2[0];
	pMomentsTgt->S2[1] = pMoments1->S2[1] + pMoments2->S2[1];
	pMomentsTgt->S2[2] = pMoments1->S2[2] + pMoments2->S2[2];
	pMomentsTgt->S2[3] = pMomentsTgt->S2[1];
	pMomentsTgt->S2[4] = pMoments1->S2[4] + pMoments2->S2[4];
	pMomentsTgt->S2[5] = pMoments1->S2[5] + pMoments2->S2[5];
	pMomentsTgt->S2[6] = pMomentsTgt->S2[2];
	pMomentsTgt->S2[7] = pMomentsTgt->S2[5];
	pMomentsTgt->S2[8] = pMoments1->S2[8] + pMoments2->S2[8];
}


/*
  InsideTriangle decides if a point P is Inside of the triangle
  defined by A, B, C.
*/
BOOL RVLInsideTriangle(double Ax, double Ay,
                      double Bx, double By,
                      double Cx, double Cy,
                      double Px, double Py)

{
  double ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
  double cCROSSap, bCROSScp, aCROSSbp;

  ax = Cx - Bx;  ay = Cy - By;
  bx = Ax - Cx;  by = Ay - Cy;
  cx = Bx - Ax;  cy = By - Ay;
  apx= Px - Ax;  apy= Py - Ay;
  bpx= Px - Bx;  bpy= Py - By;
  cpx= Px - Cx;  cpy= Py - Cy;

  aCROSSbp = ax*bpy - ay*bpx;
  cCROSSap = cx*apy - cy*apx;
  bCROSScp = bx*cpy - by*cpx;

  return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
}

// C3D(3x3) = J(3x2) * C2D(2x2) * J'(3x2)

void RVLGetCov3DFromCov2D(double *C2D, double *J, double *C3D)
{
	// M(3x2) = J(3x2) * C2D(2x2)

	double M[6];
	
	M[0 * 2 + 0] = J[0 * 2 + 0] * C2D[0] + J[0 * 2 + 1] * C2D[1];
	M[0 * 2 + 1] = J[0 * 2 + 0] * C2D[1] + J[0 * 2 + 1] * C2D[3];

	M[1 * 2 + 0] = J[1 * 2 + 0] * C2D[0] + J[1 * 2 + 1] * C2D[1];
	M[1 * 2 + 1] = J[1 * 2 + 0] * C2D[1] + J[1 * 2 + 1] * C2D[3];

	M[2 * 2 + 0] = J[2 * 2 + 0] * C2D[0] + J[2 * 2 + 1] * C2D[1];
	M[2 * 2 + 1] = J[2 * 2 + 0] * C2D[1] + J[2 * 2 + 1] * C2D[3];

	// C2D(3x3) = M(3x2) * J'(3x2)

	C3D[0 * 3 + 0] = M[0 * 2 + 0] * J[0 * 2 + 0] + M[0 * 2 + 1] * J[0 * 2 + 1];
	C3D[0 * 3 + 1] = M[0 * 2 + 0] * J[1 * 2 + 0] + M[0 * 2 + 1] * J[1 * 2 + 1];
	C3D[0 * 3 + 2] = M[0 * 2 + 0] * J[2 * 2 + 0] + M[0 * 2 + 1] * J[2 * 2 + 1];

	C3D[1 * 3 + 0] = M[1 * 2 + 0] * J[0 * 2 + 0] + M[1 * 2 + 1] * J[0 * 2 + 1];
	C3D[1 * 3 + 1] = M[1 * 2 + 0] * J[1 * 2 + 0] + M[1 * 2 + 1] * J[1 * 2 + 1];
	C3D[1 * 3 + 2] = M[1 * 2 + 0] * J[2 * 2 + 0] + M[1 * 2 + 1] * J[2 * 2 + 1];

	C3D[2 * 3 + 0] = M[2 * 2 + 0] * J[0 * 2 + 0] + M[2 * 2 + 1] * J[0 * 2 + 1];
	C3D[2 * 3 + 1] = M[2 * 2 + 0] * J[1 * 2 + 0] + M[2 * 2 + 1] * J[1 * 2 + 1];
	C3D[2 * 3 + 2] = M[2 * 2 + 0] * J[2 * 2 + 0] + M[2 * 2 + 1] * J[2 * 2 + 1];
}

// C2D(2x2) = J(2x3) * C3D(3x3) * J'(2x3)

void RVLGetCov2DFromCov3D(double *C3D, double *J, double *C2D)
{
	// M(2x3) = J(2x3) * C2D(3x3)

	double M[6];
	
	M[0 * 3 + 0] = J[0 * 3 + 0] * C3D[0 * 3 + 0] + 
				   J[0 * 3 + 1] * C3D[1 * 3 + 0] + 
				   J[0 * 3 + 2] * C3D[2 * 3 + 0];
	M[0 * 3 + 1] = J[0 * 3 + 0] * C3D[0 * 3 + 1] + 
				   J[0 * 3 + 1] * C3D[1 * 3 + 1] + 
				   J[0 * 3 + 2] * C3D[2 * 3 + 1];
	M[0 * 3 + 2] = J[0 * 3 + 0] * C3D[0 * 3 + 2] + 
				   J[0 * 3 + 1] * C3D[1 * 3 + 2] + 
				   J[0 * 3 + 2] * C3D[2 * 3 + 2];

	M[1 * 3 + 0] = J[1 * 3 + 0] * C3D[0 * 3 + 0] + 
				   J[1 * 3 + 1] * C3D[1 * 3 + 0] + 
				   J[1 * 3 + 2] * C3D[2 * 3 + 0];
	M[1 * 3 + 1] = J[1 * 3 + 0] * C3D[0 * 3 + 1] + 
				   J[1 * 3 + 1] * C3D[1 * 3 + 1] + 
				   J[1 * 3 + 2] * C3D[2 * 3 + 1];
	M[1 * 3 + 2] = J[1 * 3 + 0] * C3D[0 * 3 + 2] + 
				   J[1 * 3 + 1] * C3D[1 * 3 + 2] + 
				   J[1 * 3 + 2] * C3D[2 * 3 + 2];

	// C2D(2x2) = M(3x2) * J'(2x3)

	C2D[0 * 2 + 0] = M[0 * 3 + 0] * J[0 * 3 + 0] + 
					 M[0 * 3 + 1] * J[0 * 3 + 1] +
					 M[0 * 3 + 2] * J[0 * 3 + 2];
	C2D[0 * 2 + 1] = M[0 * 3 + 0] * J[1 * 3 + 0] + 
					 M[0 * 3 + 1] * J[1 * 3 + 1] +
					 M[0 * 3 + 2] * J[1 * 3 + 2];

	C2D[1 * 2 + 0] = M[1 * 3 + 0] * J[0 * 3 + 0] + 
					 M[1 * 3 + 1] * J[0 * 3 + 1] +
					 M[1 * 3 + 2] * J[0 * 3 + 2];
	C2D[1 * 2 + 1] = M[1 * 3 + 0] * J[1 * 3 + 0] + 
					 M[1 * 3 + 1] * J[1 * 3 + 1] +
					 M[1 * 3 + 2] * J[1 * 3 + 2];
}

// Ctgt(m x m) = J(m x n) * Csrc(n x n) * J(m x n)'
// Tmp(m x n) is auxilliary matrix 
// Csrc and Ctgt can be the same matrix

void RVLCovTransf(CvMat *Csrc, 
				  CvMat *J,
				  CvMat *Ctgt,
				  CvMat *Tmp)
{
	cvMatMul(J, Csrc, Tmp);
	cvGEMM(Tmp, J, 1.0, NULL, 1.0, Ctgt, CV_GEMM_B_T);	
}

// C1D = J(1x3) * C3D(3x3) * J'(1x3)

double RVLGetCov1DFromCov3D(double *C3D, double *J)
{
	// V(3x1) = C3D'(3x3) * J'(1x3)

	double V[3];
	
	LinearTransform3DT(C3D, J, V);

	// C1D = V'(3x1) * J'(1x3)

	return RVLDotProduct(V, J);
}

// dM = dX' * inv(C3D) * dX

double RVLMahalanobisDistance3D(double *dX, double *C3D)
{
	double invC3D[9];

	InverseMatrix3(invC3D, C3D);

	return RVLGetCov1DFromCov3D(invC3D, dX);
}

// dM = dX' * inv(C) * dX

double RVLMahalanobisDistance(CvMat *dX, CvMat *C, CvMat *Tmp)
{
	if(cvSolve(C, dX, Tmp))
		return cvDotProduct(dX, Tmp);
	else
		return -1.0;
}

// C1D = J(1x2) * C2D(2x2) * J'(1x2)

double RVLGetCov1DFromCov2D(double *C2D, double *J)
{
	return C2D[0] * J[0] * J[0] + (C2D[1] + C2D[2]) * J[0] * J[1] + C2D[3] * J[1] * J[1];	
}

// COut(n x n) = J(n x m) * CIn(m x m) * J(n x m)'

void RVLCovTransf(double *CIn, double *J, int n, int m, double *COut)
{
	double *M = new double [m * n];

	MatrixMultiplication(J, CIn, M, n, m, m);

	MatrixMultiplicationT(M, J, COut, n, m, n);

	delete[] M;
}

// Y(nxn) = X'(nx1) * X(nx1)

void RVLVector2Matrix(double *X, int n, double *Y)
{
	int i, j;

	for(i = 0; i < n; i++)
	{
		for(j = 0; j < i; j++)
			Y[i * n + j] = Y[j * n + i];

		for(j = i; j < n; j++)
			Y[i * n + j] = X[i] * X[j];
	}
};

void RVLCompleteCov(double *C, int n)
{
	int i, j;

	for(i = 1; i < n; i++)
		for(j = 0; j < i; j++)
			C[i * n + j] = C[j * n + i];
};

void RVLPrintMatrix(FILE *fp, double *A, int n, int m)
{
	double *pA = A;

	int i, j;

	for(i = 0; i < n; i++)
	{
		for(j = 0; j < m; j++, pA++)
			fprintf(fp, "%lf\t", *pA);

		fprintf(fp, "\n");
	}
}

// Function RVLGaussRand() is implemented according to the code given at
// http://www.bearcave.com/misl/misl_tech/wavelets/hurst/random.html

double RVLGaussRand(double std)
{
	double x1, x2, w;

	do
	{
		x1 = 2.0 * ((double)rand() / (double)RAND_MAX) - 1.0;
		x2 = 2.0 * ((double)rand() / (double)RAND_MAX) - 1.0;

		w = x1 * x1 + x2 * x2;
	}
	while(w >= 1.0 || w < APPROX_ZERO);

	w = sqrt( (-2.0 * log( w ) ) / w );

    return std * x1 * w;
}

// Function RVLGaussRandBM() is implemented according to the code given at
// http://en.wikipedia.org/wiki/Box-Muller_transform

double RVLGaussRandBM(double std)
{
	return std * sqrt(-2.0 * log((double)rand() / (double)RAND_MAX)) * 
		cos(2.0 * PI * ((double)rand() / (double)RAND_MAX));
}

void RVLResize(PIX_ARRAY *pPixArray, 
			   int Size)
{
	if(pPixArray->bOwnData)
	{
		if(pPixArray->Width * pPixArray->Height == Size)
			return;

		if(pPixArray->pPix)
			delete[] pPixArray->pPix;
	}
	else
		pPixArray->bOwnData = TRUE;

	pPixArray->pPix = new unsigned char[Size];
}

void RVLRandomSelect(void **SrcArray, 
					 int n,
					 int m,
					 void **TgtArray)
{
	int nRem = n;

	void **pTgtEnd = TgtArray + m;

	void **pTgt;

	int i;

	for(pTgt = TgtArray; pTgt < pTgtEnd; pTgt++)
	{
		i = RVLRandom(0, nRem - 1);

		*pTgt = SrcArray[i];

		if(i < nRem - 1)
			memcpy(SrcArray + i, SrcArray + i + 1, sizeof(void *) * (nRem - i - 1));

		nRem--;
	}
}
/*
void **RVLCreatePtrArray(CRVLMPtrChain *pSrcArray,
		  	             CRVLMem *pMem)

{
	void **TgtArray = (void **)(pMem->Alloc(pSrcArray->m_nElements * sizeof(void *)));

	void **pTgt = TgtArray;

	pSrcArray->Start();

	while(pSrcArray->m_pNext)
		*(pTgt++) = pSrcArray->GetNext();

	return TgtArray;
}*/

void RVLPrintCov(FILE *fp,
				 CvMat *C,
				 CvMat *W,
				 CvMat *U,
				 double k)
{
	cvSVD(C, W, U);

	int m = C->cols;

	int i, j;

	for(i = 0; i < C->rows; i++)
	{
		for(j = 0; j < m; j++)
			fprintf(fp, "%lf\t", U->data.db[m * j + i]);

		fprintf(fp, "|%lf\n", k * sqrt(W->data.db[i]));
	}
}

void RVLPrintCov(FILE *fp,
				 double *CIn,
				 int n,
				 double k)
{
	CvMat *C = cvCreateMatHeader(n, n, CV_64FC1);
	C->data.db = CIn;
	CvMat *W = cvCreateMat(n, 1, CV_64FC1);
	CvMat *U = cvCreateMat(n, n, CV_64FC1);

	RVLPrintCov(fp, C, W, U, k);

	cvReleaseMat(&C);
	cvReleaseMat(&W);
	cvReleaseMat(&U);
}

void RVLInitMatrices()
{
	RVLMatrix31 = cvCreateMat(3, 1, CV_64FC1);
	RVLMatrix33 = cvCreateMat(3, 3, CV_64FC1);

	RVLMatrixHeader31 = cvCreateMatHeader(3, 1, CV_64FC1);
	RVLMatrixHeaderA33 = cvCreateMatHeader(3, 3, CV_64FC1);
	RVLMatrixHeaderB33 = cvCreateMatHeader(3, 3, CV_64FC1);
	RVLMatrixHeaderC33 = cvCreateMatHeader(3, 3, CV_64FC1);
}

void RVLClearMatrices()
{
	cvReleaseMat(&RVLMatrix31);
	cvReleaseMat(&RVLMatrix33);

	cvReleaseMat(&RVLMatrixHeader31);
	cvReleaseMat(&RVLMatrixHeaderA33);
	cvReleaseMat(&RVLMatrixHeaderB33);
	cvReleaseMat(&RVLMatrixHeaderC33);
}
/*
void **RVLChainToArray(CRVLMPtrChain *pChain, 
		   			   CRVLMem *pMem,
					   int &n)
{
	n = pChain->m_nElements;

	void **Array = (void **)(pMem->Alloc(sizeof(void *) * n));

	void **p = Array;

	pChain->Start();

	while(pChain->m_pNext)
		*(p++) = pChain->GetNext();

	return Array;
}*/


int RVLSqrDistance2LineSeg(	int *iU, 
							int *iU1, int *iU2)
{
	int p, q;

	int du = iU2[0] - iU1[0];
	int dv = iU2[1] - iU1[1];
	int len = DOUBLE2INT(sqrt((double)(du*du+dv*dv)));

	LinearTransform2D(du, dv, len, iU1[0], iU1[1], iU[0], iU[1], p, q);

	if(p < 0)
	{
		du = iU[0] - iU1[0];
		dv = iU[1] - iU1[1];

		return du*du+dv*dv;
	}
	else if(p > len)
	{
		du = iU[0] - iU2[0];
		dv = iU[1] - iU2[1];

		return du*du+dv*dv;
	}
	else
		return q*q;	
}

void RVLGrayscaleToRGB(unsigned char *pPixSrc,
					   int w, int h,
					   unsigned char *pPixTgt)
{
	unsigned char *pPixIn = pPixSrc;
	unsigned char *pPixOut = pPixTgt;

	int n = w * h;

	unsigned char I;
	int iPix;

	for(iPix = 0; iPix < n; iPix++, pPixIn++)
	{
		I = (*pPixIn);

		*(pPixOut++) = I;
		*(pPixOut++) = I;
		*(pPixOut++) = I;
	}
}

void RVLCreateEmptyRegionGrowingMap(unsigned short *RegionGrowingMap,
									int w, int h,
									BYTE Empty,
									BYTE Border)
{
	int RowSize = w * sizeof(unsigned short);

	memset(RegionGrowingMap, Border, RowSize);

	memset(RegionGrowingMap + w, Empty, (h - 2) * RowSize);

	memset(RegionGrowingMap + (h - 1) * w, 0xfe, RowSize);

	unsigned short Border2 = (((unsigned short)Border << 16) | (unsigned short)Border);

	int v;

	for(v = 1; v < h - 1; v++)
		RegionGrowingMap[v * w] = RegionGrowingMap[(v + 1) * w - 1] = Border2;
}

char *RVLCreateString(char *strIn)
{
	char *strOut = new char[strlen(strIn) + 1];

	strcpy(strOut, strIn);

	return strOut;
}

void RVLCopyString(char *strSrc, char **pstrTgt, CRVLMem *pMem)
{
	int lenSrc = strlen(strSrc);

	char *strTgt = *pstrTgt;

	int lenTgt = (strTgt ? strlen(strTgt) : 0);

	BOOL bMemAlloc;

	if(strTgt)
	{
		if(bMemAlloc = (lenSrc > lenTgt))
			if(pMem == NULL)
				delete[] strTgt;
	}
	else
		bMemAlloc = TRUE;

	if(bMemAlloc)
	{
		if(pMem)
			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, char, lenSrc + 1, *pstrTgt)
		else
			*pstrTgt = new char[lenSrc + 1];
	}

	strTgt = *pstrTgt;

	strcpy(strTgt, strSrc);
}

char *RVLCreateFileName(char *SrcFileName, 
						char *SrcExtension,
						int n,
						char *TgtExtension,
						CRVLMem *pMem)
{
	char *TgtFileName;

	int LenTgt = strlen(SrcFileName) - strlen(SrcExtension) + strlen(TgtExtension) + 1;

	if(pMem)
		RVLMEM_ALLOC_STRUCT_ARRAY(pMem, char, LenTgt, TgtFileName)
	else
		TgtFileName = new char[LenTgt];

	int NumberPosition = strlen(SrcFileName) - strlen(SrcExtension) - 5;

	if(n >= 0)
	{
		memcpy(TgtFileName, SrcFileName, NumberPosition + 1);

		sprintf(TgtFileName + NumberPosition, "%05d", n);
	}
	else
		memcpy(TgtFileName, SrcFileName, NumberPosition + 6);

	strcpy(TgtFileName + NumberPosition + 5, TgtExtension);

	return TgtFileName;
}

void RVLCopyPixArray(PIX_ARRAY *pSrcImage,
					 PIX_ARRAY *pTgtImage)
{
	int w = pSrcImage->Width;
	int h = pSrcImage->Height;
	int ImageSize = w * h;

	if(pTgtImage->pPix)
	{
		int pTgtImageSize = pTgtImage->Width * pTgtImage->Height;

		if(pTgtImageSize < ImageSize)
		{
			delete[] pTgtImage->pPix;

			pTgtImage->pPix = new unsigned char[ImageSize];
		}
	}
	else
		pTgtImage->pPix = new unsigned char[ImageSize];

	pTgtImage->bOwnData = TRUE;

	pTgtImage->Width = w;
	pTgtImage->Height = h;
	pTgtImage->bColor = pSrcImage->bColor;
	pTgtImage->nPixBytes = pSrcImage->nPixBytes;

	memcpy(pTgtImage->pPix, pSrcImage->pPix, ImageSize);	
}

void RVLCopyImageToPixArray(IplImage *pSrcImage,
							PIX_ARRAY *pTgtImage)
{
	int w = pSrcImage->width;
	int h = pSrcImage->height;

	BOOL bColor = (pSrcImage->nChannels == 3);

	int nPixBytes = (bColor ? 3 : 1);

	int ImageSize = w * h * nPixBytes;

	if(pTgtImage->pPix)
	{
		int pTgtImageSize = pTgtImage->Width * pTgtImage->Height * pTgtImage->nPixBytes;

		if(pTgtImageSize < ImageSize)
		{
			delete[] pTgtImage->pPix;

			pTgtImage->pPix = new unsigned char[ImageSize];
		}
	}
	else
		pTgtImage->pPix = new unsigned char[ImageSize];

	pTgtImage->bOwnData = TRUE;

	pTgtImage->Width = w;
	pTgtImage->Height = h;
	pTgtImage->bColor = bColor;
	pTgtImage->nPixBytes = nPixBytes;

	memcpy(pTgtImage->pPix, pSrcImage->imageData, ImageSize);	
}

void Skew(double *V, double *M)
{
	M[0 * 3 + 0] = 0;
	M[0 * 3 + 1] = -V[2];
	M[0 * 3 + 2] = V[1];

	M[1 * 3 + 0] = V[2];
	M[1 * 3 + 1] = 0;
	M[1 * 3 + 2] = -V[0];

	M[2 * 3 + 0] = -V[1];
	M[2 * 3 + 1] = V[0];
	M[2 * 3 + 2] = 0;

}
int* RVLWhereAbove(int *x, int n, double l, int &j)
{
	int * idx = new int[n];
	int br = 0;
	for (int i = 0; i < n; i++)
	{
		if (x[i] > l)
		{
			idx[br] = i;
			br++;
		}
	}
	j = br;
	return idx;
}

int* RVLWhereAbove(float *x, int n, double l, int &j)
{
	int * idx = new int[n];
	int br = 0;
	for (int i = 0; i < n; i++)
	{
		if (x[i] > l)
		{
			idx[br] = i;
			br++;
		}
	}
	j = br;
	return idx;
}

int* RVLWhereAbove(double *x, int n, double l, int &j)
{
	int * idx = new int[n];
	int br = 0;
	for (int i = 0; i < n; i++)
	{
		if (x[i] > l)
		{
			idx[br] = i;
			br++;
		}
	}
	j = br;
	return idx;
}

void RVLSetFileNumber(char *FileName, char *Extension, int n)
{
	char index[6];

	sprintf(index, "%05d", n);

	memcpy(FileName + strlen(FileName) - strlen(Extension), index, 5 * sizeof(char)); 
}

int RVLGetFileNumber(char *FileName, char *Extension)
{
	return atoi(FileName + strlen(FileName) - strlen(Extension));
}

BOOL RVLGetNextFileName(char *FileName, char *Extension, int maxiSample)
{
	int iSample = RVLGetFileNumber(FileName, Extension);

	FILE *fp;

	do
	{
		iSample++;

		if(iSample > maxiSample)
			return FALSE;

		RVLSetFileNumber(FileName, Extension, iSample);

		fp = fopen(FileName, "r");
	}
	while(fp == NULL);

	fclose(fp);

	return TRUE;
}

// Converts block matrix 3x3x3 to 6x6 matrix
void RVL3x3x3BlockMxTo6x6(double *PSrc, double *PTgt)
{
	PTgt[0*6+0] = PSrc[0*9+0*3+0];
	PTgt[0*6+1] = PSrc[0*9+0*3+1];
	PTgt[0*6+2] = PSrc[0*9+0*3+2];
	PTgt[0*6+3] = PSrc[1*9+0*3+0];
	PTgt[0*6+4] = PSrc[1*9+0*3+1];
	PTgt[0*6+5] = PSrc[1*9+0*3+2];

	PTgt[1*6+0] = PSrc[0*9+1*3+0];
	PTgt[1*6+1] = PSrc[0*9+1*3+1];
	PTgt[1*6+2] = PSrc[0*9+1*3+2];
	PTgt[1*6+3] = PSrc[1*9+1*3+0];
	PTgt[1*6+4] = PSrc[1*9+1*3+1];
	PTgt[1*6+5] = PSrc[1*9+1*3+2];

	PTgt[2*6+0] = PSrc[0*9+2*3+0];
	PTgt[2*6+1] = PSrc[0*9+2*3+1];
	PTgt[2*6+2] = PSrc[0*9+2*3+2];
	PTgt[2*6+3] = PSrc[1*9+2*3+0];
	PTgt[2*6+4] = PSrc[1*9+2*3+1];
	PTgt[2*6+5] = PSrc[1*9+2*3+2];

	PTgt[3*6+0] = PSrc[1*9+0*3+0];
	PTgt[3*6+1] = PSrc[1*9+1*3+0];
	PTgt[3*6+2] = PSrc[1*9+2*3+0];
	PTgt[3*6+3] = PSrc[2*9+0*3+0];
	PTgt[3*6+4] = PSrc[2*9+0*3+1];
	PTgt[3*6+5] = PSrc[2*9+0*3+2];
			   
	PTgt[4*6+0] = PSrc[1*9+0*3+1];
	PTgt[4*6+1] = PSrc[1*9+1*3+1];
	PTgt[4*6+2] = PSrc[1*9+2*3+1];
	PTgt[4*6+3] = PSrc[2*9+1*3+0];
	PTgt[4*6+4] = PSrc[2*9+1*3+1];
	PTgt[4*6+5] = PSrc[2*9+1*3+2];
			   
	PTgt[5*6+0] = PSrc[1*9+0*3+2];
	PTgt[5*6+1] = PSrc[1*9+1*3+2];
	PTgt[5*6+2] = PSrc[1*9+2*3+2];
	PTgt[5*6+3] = PSrc[2*9+2*3+0];
	PTgt[5*6+4] = PSrc[2*9+2*3+1];
	PTgt[5*6+5] = PSrc[2*9+2*3+2];
}

// Given 3DOF transformation represented by the orientation angle alphaAB and translation vector [txAB tyAB]' and its 
// covariance matrix C, the following function computes the covariance matirx invC of the inverse transformation 
// represented by the orientation angle alphaBA and translation vector [txBA tyBA]'.
// NOTE: The function requires txBA and tyBA as input parameters!
//       The funciton uses RVLMatrixA33
// The mathematics for this function is given in RVLMath.doc, chapter "Pose Uncertainty Propagation"


void RVL3DOFInvTransfUncert(double csalphaAB, 
							double snalphaAB,
							double txBA,
							double tyBA,
							double *C,
							double *invC)
{
	RVLMatrixA33[0] = -C[0];
	RVLMatrixA33[1] = -C[1];
	RVLMatrixA33[2] = -C[2];
	RVLMatrixA33[3] =  tyBA * C[0] - csalphaAB * C[3] - snalphaAB * C[6];
	RVLMatrixA33[4] =  tyBA * C[1] - csalphaAB * C[4] - snalphaAB * C[7];
	RVLMatrixA33[5] =  tyBA * C[2] - csalphaAB * C[5] - snalphaAB * C[8];
	RVLMatrixA33[6] = -txBA * C[0] + snalphaAB * C[3] - csalphaAB * C[6];
	RVLMatrixA33[7] = -txBA * C[1] + snalphaAB * C[4] - csalphaAB * C[7];
	RVLMatrixA33[8] = -txBA * C[2] + snalphaAB * C[5] - csalphaAB * C[8];

	invC[0] = -RVLMatrixA33[0];
	invC[1] =  tyBA * RVLMatrixA33[0] - csalphaAB * RVLMatrixA33[1] - snalphaAB * RVLMatrixA33[2];
	invC[2] = -txBA * RVLMatrixA33[0] + snalphaAB * RVLMatrixA33[1] - csalphaAB * RVLMatrixA33[2];
	invC[3] = -RVLMatrixA33[3];
	invC[4] =  tyBA * RVLMatrixA33[3] - csalphaAB * RVLMatrixA33[4] - snalphaAB * RVLMatrixA33[5];
	invC[5] = -txBA * RVLMatrixA33[3] + snalphaAB * RVLMatrixA33[4] - csalphaAB * RVLMatrixA33[5];
	invC[6] = -RVLMatrixA33[6];
	invC[7] =  tyBA * RVLMatrixA33[6] - csalphaAB * RVLMatrixA33[7] - snalphaAB * RVLMatrixA33[8];
	invC[8] = -txBA * RVLMatrixA33[6] + snalphaAB * RVLMatrixA33[7] - csalphaAB * RVLMatrixA33[8];
}

void RVLZoom(IplImage *pSrcImage, IplImage *pTgtImage, int ZoomFactor)
{
	char *pSrcPixRow = pSrcImage->imageData;

	int SrcRowSize = pSrcImage->widthStep;

	char *pTgtPixRow = pTgtImage->imageData;
	int TgtRowSize = pTgtImage->widthStep;

	int u, v;
	int i, j, k;
	char *pSrcPix, *pTgtPix, *pPrevTgtPixRow;

	for(v = 0; v < pSrcImage->height; v++)
	{
		pSrcPix = pSrcPixRow;
		pTgtPix = pTgtPixRow;

		for(u = 0; u < pSrcImage->width; u++, pSrcPix += pSrcImage->nChannels)
			for(i = 0; i < ZoomFactor; i++)
				for(j = 0; j < pSrcImage->nChannels; j++)
					*(pTgtPix++) = pSrcPix[j];

		pPrevTgtPixRow = pTgtPixRow;

		pTgtPixRow += TgtRowSize;

		for(k = 1; k < ZoomFactor; k++, pTgtPixRow += TgtRowSize)
			memcpy(pTgtPixRow, pPrevTgtPixRow, TgtRowSize);

		pSrcPixRow += SrcRowSize;
	}
}

double RVLTriangularPDF(double val,
						double Val0,
						double mindVal,
						double maxdVal)
{
	double e = val - Val0;

	if(e < mindVal)
		return 0.0;
	else if(e < 0.0)
		return (mindVal - e) / mindVal;
	else if(e <= maxdVal)
		return (maxdVal - e) / maxdVal;
	else
		return 0.0;
}

double RVLBilateralGaussianPDF(	double val,
								double Val0,
								double mindVal,
								double maxdVal)
{
	double sig = (val >= Val0 ? maxdVal : mindVal);

	double e = (val - Val0) / sig;

	return exp(-0.5 * e * e);
}

double RVLBilateralGaussianPDFAngle(double val,
									double Val0,
									double mindVal,
									double maxdVal)
{
	double sig = (val >= Val0 ? maxdVal : mindVal);

	double e = val - Val0;

	if(e > PI)
		e -= (2.0 * PI);
	else if(e < -PI)
		e += (2.0 * PI);

	e /= sig;

	return exp(-0.5 * e * e);
}

double RVLLogBilateralGaussianPDF(	double val,
									double Val0,
									double mindVal,
									double maxdVal)
{
	double sig = (val >= Val0 ? maxdVal : -mindVal);

	double e = (val - Val0) / sig;

	return -0.5 * e * e;
}

double RVLLogBilateralGaussianPDFAngle(double val,
									double Val0,
									double mindVal,
									double maxdVal)
{
	double sig = (val >= Val0 ? maxdVal : -mindVal);

	double e = val - Val0;

	if(e > PI)
		e -= (2.0 * PI);
	else if(e < -PI)
		e += (2.0 * PI);

	e /= sig;

	return -0.5 * e * e;
}

double RVLBilateralGaussianPDF(	double val,
								void *vpParams)
{
	double *Param = (double *)vpParams;

	return RVLBilateralGaussianPDF(val, Param[0], Param[1], Param[2]);
}



void RVLRandPDFLT(	double PDF(double val, void *vpParams),
					void *vpParams_,
					double min,
					double max,
					double dval,
					double nrm,
					RVLRANDPDFLT *pLT)
{
	int imin = pLT->min = DOUBLE2INT(min / dval);
	int imax = pLT->max = DOUBLE2INT(max / dval);
	pLT->nrm = nrm;
	pLT->dval;

	int n = 0;

	int i;

	for(i = imin; i <= imax; i++)
		n += (DOUBLE2INT(PDF((double)i * dval, vpParams_) * nrm));

	double *LT = new double[n];

	i = 0;

	int j, m;

	while(i < n)
	{
		m = DOUBLE2INT(PDF((double)i * dval, vpParams_) * nrm);

		for(j = 0; j < m; j++, i++)
			LT[i] = (double)(i + imin) * dval;
	}

	pLT->LT = LT;
	pLT->n = n;
}

double RVLRandPDF(RVLRANDPDFLT *pLT)
{
	return pLT->LT[RVLRandom(0, pLT->n - 1)];
}

double RVLRandPDF(	double PDF(double val_, void *vpParams_),
					void *vpParams,
					double min,
					double max,
					double dval)
{
	int imin = DOUBLE2INT(min / dval);
	int imax = DOUBLE2INT(max / dval);	

	double range = 0.0;

	int i;

	for(i = imin; i <= imax; i++)
		range += PDF((double)i * dval, vpParams);

	double cumsum_ref = RVLRandom(0.0, range);

	double cumsum = 0.0;

	int k = imin;

	double x;

	while(cumsum <= cumsum_ref)
	{
		x = (double)k * dval;

		cumsum += PDF(x, vpParams);

		k++;
	}

	return x;
}


// RVLCamera.cpp: implementation of the CRVLCamera class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "Platform.h"

#include "RVLCore.h"
#include "RVLCamera.h"


//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLCamera::CRVLCamera()
{
	m_Flags = 0x00000000;
	m_Image.pPix = NULL;
	m_nrmImage.pPix = NULL;
	m_nrmud = NULL;
	m_nrmvd = NULL;
	m_pRGBImage = NULL;
}

CRVLCamera::~CRVLCamera()
{
	if(m_Image.bOwnData && m_Image.pPix != NULL)
		delete[] m_Image.pPix;	

	if(m_nrmImage.bOwnData && m_nrmImage.pPix != NULL)
		delete[] m_nrmImage.pPix;

	if(m_nrmud != NULL)
		delete[] m_nrmud;

	if(m_nrmvd != NULL)
		delete[] m_nrmvd;

	if(m_pRGBImage)
		cvReleaseImage(&m_pRGBImage);
}

DWORD CRVLCamera::Init(char *CameraParamsFileName)
{
	DWORD ret = LoadParams(CameraParamsFileName);

	if(ret != RVL_RES_OK)
		return ret;

	int ImageSize = Width * Height;

	m_Image.Width = Width;
	m_Image.Height = Height;
	m_Image.bOwnData = FALSE;
	m_Image.pPix = NULL;
	m_Image.bColor = FALSE;

	m_nrmImage.Width = Width;
	m_nrmImage.Height = Height;
	m_nrmImage.bOwnData = TRUE;
	m_nrmImage.pPix = new unsigned char[ImageSize];
	m_nrmImage.bColor = FALSE;

	return RVL_RES_OK;
}


void CRVLCamera::GetProjectionMatrix()
{
	double K[9];

	GetKMatrix(K, f, sx, dpx, dpy, CenterX, CenterY);

	MatrixMultiplication(K, T, PR, 3, 3);
	LinearTransform3D(K, T + 9, PR + 9);

	InverseMatrix3(InvPR, PR);
	MatrixMultiplication(InvPR, PR + 9, InvPR + 9, 3, 3);
}

void CRVLCamera::CreateImageNrmLookupTable(int uShift)
{
	int ImageSize = Width * Height;

	m_nrmud = new short int[ImageSize];
	m_nrmvd = new short int[ImageSize];

	int iPix = 0;
	double *T = NrmTransf;

	int uFirst, uLast, iPixOffset;

	if(uShift > 0)
	{
		uFirst = 0;
		uLast = Width - uShift;
		iPixOffset = uShift;
	}
	else
	{
		uFirst = -uShift;
		uLast = Width;
		iPixOffset = 0;
	}

	int u, v, ud, vd;
	double fud, fvd, fuu, fvu;
	double P11, P12, P21, P22, Q1, Q2, detP;
	double fuNrm, fvNrm;

	for(v = 0; v < Height; v++)
	{
		if(uShift > 0)
		{
			memset(m_nrmud + iPix, 0, uShift * sizeof(short int));
			memset(m_nrmvd + iPix, 0, uShift * sizeof(short int));

			iPix += uShift;
		}
	
		for(u = uFirst; u < uLast; u++)
		{
			fuNrm = (double)u;
			fvNrm = (double)v;

			P11 = T[2 * 3 + 0] * fuNrm - T[0 * 3 + 0];
			P12 = T[2 * 3 + 1] * fuNrm - T[0 * 3 + 1];
			P21 = T[2 * 3 + 0] * fvNrm - T[1 * 3 + 0];
			P22 = T[2 * 3 + 1] * fvNrm - T[1 * 3 + 1];
			Q1 = T[2 * 3 + 2] * fuNrm - T[0 * 3 + 2];
			Q2 = T[2 * 3 + 2] * fvNrm - T[1 * 3 + 2];

			detP = P11 * P22 - P12 * P21;

			fuu = (-P22 * Q1 + P12 * Q2) / detP;
			fvu = ( P21 * Q1 - P11 * Q2) / detP;

			Distortion(fuu, fvu, fud, fvd);

			ud = DOUBLE2INT(fud * 32.0);
			vd = DOUBLE2INT(fvd * 32.0);

			if(ud >= 1 && ud <= ((Width - 2) << 5) && 
				vd >= 1 && vd <= ((Height - 2) << 5))
			{
				m_nrmud[iPix] = (short int)ud;
				m_nrmvd[iPix] = (short int)vd;
			}
			else
			{
				m_nrmud[iPix] = 0;
				m_nrmvd[iPix] = 0;
			}

			iPix++;
		}

		if(uShift < 0)
		{
			memset(m_nrmud + iPix, 0, -uShift * sizeof(short int));
			memset(m_nrmvd + iPix, 0, -uShift * sizeof(short int));

			iPix -= uShift;
		}
	}
}

void CRVLCamera::Distortion(double xu, double yu, double &xd, double &yd)
{
	double xu2 = (xu - CenterX);
	double yu2 = (yu - CenterY);
	double xu3 = xu2 * dpx / sx;
	double yu3 = yu2 * dpy;
	double Ru2 = xu3 * xu3 + yu3 * yu3;
	double Ru = sqrt(Ru2);
	double k1 = k;
	double k2;

	if(k > 1e-10)
	{
		double t = sqrt(Ru2 + 4.0 / 27.0 / k1);
		double Rd = pow((Ru + t) * 0.5 / k1, 1.0 / 3.0) - pow((-Ru + t) * 0.5 / k1, 1.0 / 3.0);
		k2 = 1.0 / (1.0 + k1 * Rd * Rd);
	}
	else
		k2 = 1.0;

	xd = xu2 * k2 + CenterX;
	yd = yu2 * k2 + CenterY;
}

void CRVLCamera::NormalizeImage()
{
	m_Image.pPix[0] = 0;

	unsigned char *pITgt = m_nrmImage.pPix;
	short int *pUd = m_nrmud;
	short int *pVd = m_nrmvd;
	int nPix = Width * Height;

	int iPix, iPix2;
	int eu1, ev1, eu2, ev2;

	for(iPix = 0; iPix < nPix - Width; iPix++)
	{
		//*(pITgt++) = ISrc[(((*(pVd++)) + 0x10) >> 5) * Width + (((*(pUd++)) + 0x10) >> 5)];
	  
		iPix2 = ((*pVd) >> 5) * Width + ((*pUd) >> 5);

		eu1 = (*pUd & 0x1f);
		ev1 = (*pVd & 0x1f);
		eu2 = 0x20 - eu1;
		ev2 = 0x20 - ev1;

		*(pITgt++) = ((	((int)(m_Image.pPix[iPix2]) * eu2 + 
						(int)(m_Image.pPix[iPix2 + 1]) * eu1) * ev2 +
						((int)(m_Image.pPix[iPix2 + Width]) * eu2 + 
						(int)(m_Image.pPix[iPix2 + Width + 1]) * eu1) * ev1) >> 10);

		pUd++;
		pVd++;
	}
}

DWORD CRVLCamera::SetImage(PIX_ARRAY *pImage)
{
	if(pImage->Width != Width)
		return RVLCAMERA_RETURN_ERR_FALSE_IMAGE_FORMAT;

	if(pImage->Height != Height)
		return RVLCAMERA_RETURN_ERR_FALSE_IMAGE_FORMAT;

	m_Image.bOwnData = FALSE;

	m_Image.pPix = pImage->pPix;

	return RVL_RES_OK;
}

void CRVLCamera::GetParallelStereoCameraSystem(CRVLCamera *pSecondCamera)
{
	int i;
	double InvT[9];

	// Tilt angle of the Camera1

	InverseRotation(InvT, T);

	double Cp01[3];

	LinearTransform3D(InvT, T + 9, Cp01);

	double C[3];
	C[0] = CenterX;
	C[1] = CenterY;
	C[2] = 1.0;

	double InvPR1[12];

	InverseMatrix3(InvPR1, PR);

	LinearTransform3D(InvPR1, PR + 9, InvPR1 + 9);

	double G[3];

	LinearTransform3D(InvPR1, C, G);

	double TiltAngle1 = atan(G[1] - InvPR1[9 + 1] + Cp01[1]);

	//TiltAngle = TiltAngle1 * RAD2DEG;

	// Tilt angle of the Camera2

	InverseRotation(InvT, pSecondCamera->T);

	double Cp02[3];

	LinearTransform3D(InvT, pSecondCamera->T + 9, Cp02);

	C[0] = pSecondCamera->CenterX;
	C[1] = pSecondCamera->CenterY;
	C[2] = 1.0;

	double InvPR2[12];

	InverseMatrix3(InvPR2, pSecondCamera->PR);

	LinearTransform3D(InvPR2, pSecondCamera->PR + 9, InvPR2 + 9);

	LinearTransform3D(InvPR2, C, G);

	double TiltAngle2 = atan(G[1] - InvPR2[9 + 1] + Cp02[1]);

	//pSecondCamera->TiltAngle = TiltAngle2 * RAD2DEG;

	// Mean Tilt Angle

	double TiltAngle0 = (TiltAngle1 + TiltAngle2) * 0.5;

	TiltAngle = TiltAngle0 * RAD2DEG;
	pSecondCamera->TiltAngle = TiltAngle0 * RAD2DEG;

	// Normalization Matrices NT1 & NT2

	double RTilt[9];

	double cs = cos(TiltAngle0);
	double sn = sin(TiltAngle0);

	RTilt[0] = 1.0;
	RTilt[1] = 0.0;
	RTilt[2] = 0.0;
	RTilt[3] = 0.0;
	RTilt[4] = cs;
	RTilt[5] = -sn;
	RTilt[6] = 0.0;
	RTilt[7] = sn;
	RTilt[8] = cs;

	double InvRTilt[9];

	InverseRotation(InvRTilt, RTilt);

	double NT1[9];

	MatrixMultiplication(RTilt, InvPR1, NT1, 3, 3);

	double NT2[9];

	MatrixMultiplication(RTilt, InvPR2, NT2, 3, 3);

	// Params. Top, Bottom, Left and Right of Camera1

	double TLo[3], TRo[3], BLo[3], BRo[3];

	double fWidth = (double)Width;
	double fHeight = (double)Height;

	DistortionCorrection(0.0,   0.0,    TLo[0], TLo[1]);
	DistortionCorrection(fWidth, 0.0,    TRo[0], TRo[1]);
	DistortionCorrection(0.0,   fHeight, BLo[0], BLo[1]);
	DistortionCorrection(fWidth, fHeight, BRo[0], BRo[1]);

	TLo[2] = TRo[2] = BLo[2] = BRo[2] = 1.0;

	double TLn[3], TRn[3], BLn[3], BRn[3];

	LinearTransform3D(NT1, TLo, TLn);
	LinearTransform3D(NT1, TRo, TRn);
	LinearTransform3D(NT1, BLo, BLn);
	LinearTransform3D(NT1, BRo, BRn);

	TLn[0] /= TLn[2];
	TLn[1] /= TLn[2];
	TRn[0] /= TRn[2];
	TRn[1] /= TRn[2];
	BLn[0] /= BLn[2];
	BLn[1] /= BLn[2];
	BRn[0] /= BRn[2];
	BRn[1] /= BRn[2];

	double Top1, Bottom1, Left1, Right1;

	Top1    = (TLn[1] < TRn[1] ? TLn[1] : TRn[1]);
	Bottom1 = (BLn[1] > BRn[1] ? BLn[1] : BRn[1]);
	Left1	= (TLn[0] < BLn[0] ? TLn[0] : BLn[0]);
	Right1	= (TRn[0] > BRn[0] ? TRn[0] : BRn[0]);

	// Params. Top, Bottom, Left and Right of Camera2

	fWidth =  (double)(pSecondCamera->Width);
	fHeight = (double)(pSecondCamera->Height);

	pSecondCamera->DistortionCorrection(0.0,   0.0,    TLo[0], TLo[1]);
	pSecondCamera->DistortionCorrection(fWidth, 0.0,    TRo[0], TRo[1]);
	pSecondCamera->DistortionCorrection(0.0,   fHeight, BLo[0], BLo[1]);
	pSecondCamera->DistortionCorrection(fWidth, fHeight, BRo[0], BRo[1]);

	TLo[2] = TRo[2] = BLo[2] = BRo[2] = 1.0;

	LinearTransform3D(NT2, TLo, TLn);
	LinearTransform3D(NT2, TRo, TRn);
	LinearTransform3D(NT2, BLo, BLn);
	LinearTransform3D(NT2, BRo, BRn);

	TLn[0] /= TLn[2];
	TLn[1] /= TLn[2];
	TRn[0] /= TRn[2];
	TRn[1] /= TRn[2];
	BLn[0] /= BLn[2];
	BLn[1] /= BLn[2];
	BRn[0] /= BRn[2];
	BRn[1] /= BRn[2];

	double Top2, Bottom2, Left2, Right2;

	Top2    = (TLn[1] < TRn[1] ? TLn[1] : TRn[1]);
	Bottom2 = (BLn[1] > BRn[1] ? BLn[1] : BRn[1]);
	Left2	= (TLn[0] < BLn[0] ? TLn[0] : BLn[0]);
	Right2	= (TRn[0] > BRn[0] ? TRn[0] : BRn[0]);

	// Common Top and Bottom

	double Top = (Top1 < Top2 ? Top1 : Top2);
	double Bottom = (Bottom1 > Bottom2 ? Bottom1 : Bottom2);

	// Norm Params.

	double H = Bottom - Top;
	double W1 = Right1 - Left1;
	double W2 = Right2 - Left2;
	double W = (W1 > W2 ? W1 : W2);

	double fNW = fWidth / W;
	double fNH = fHeight / H;

	double fN = (fNW < fNH ? fNW : fNH);

	fNrm = fN;
	pSecondCamera->fNrm = fN;

	double CxN1 = (fWidth - (Left1 + Right1) * fN) * 0.5 + 64.0/*(double)m_u1Shift*/;
	double CxN2 = (fWidth - (Left2 + Right2) * fN) * 0.5;
	double CyN = (fHeight - (Top + Bottom) * fN) * 0.5;

	CenterXNrm = CxN1;
	pSecondCamera->CenterXNrm = CxN2;
	CenterYNrm = CyN;
	pSecondCamera->CenterYNrm = CyN;

	double KN1[12];

	GetKMatrix(KN1, fN, 1.0, 1.0, 1.0, CxN1, CyN);

	double KN2[12];

	GetKMatrix(KN2, fN, 1.0, 1.0, 1.0, CxN2, CyN);

	MatrixMultiplication(KN1, NT1, NrmTransf, 3, 3);

	MatrixMultiplication(KN2, NT2, pSecondCamera->NrmTransf, 3, 3);

	for(i = 0; i < 9; i++)	
		TNrm[i] = pSecondCamera->TNrm[i] = RTilt[i];

	LinearTransform3D(RTilt, Cp01, TNrm + 9);
	LinearTransform3D(RTilt, Cp02, pSecondCamera->TNrm + 9);

	// Rotate for tilt angle

	//MatrixMultiplication(KN1, TNrm, PRNrm, 3, 3);
	LinearTransform3D(KN1, TNrm + 9, PRNrm + 9);

	// Don't rotate for tilt angle

	memcpy(PRNrm, KN1, 9 * sizeof(double));
	//LinearTransform3D(KN1, Cp01, PRNrm + 9);

	/////

	InverseMatrix3(InvPRNrm, PRNrm);
	LinearTransform3D(InvPRNrm, PRNrm + 9, InvPRNrm + 9);

	P3DC0Nrm[0] = -InvPRNrm[9];
	P3DC0Nrm[1] = -InvPRNrm[10];
	P3DC0Nrm[2] = -InvPRNrm[11];

	// Rotate for tilt angle

	//MatrixMultiplication(KN2, pSecondCamera->TNrm, pSecondCamera->PRNrm, 3, 3);
	LinearTransform3D(KN2, pSecondCamera->TNrm + 9, pSecondCamera->PRNrm + 9);

	// Don't rotate for tilt angle

	memcpy(pSecondCamera->PRNrm, KN2, 9 * sizeof(double));
	//LinearTransform3D(KN2, Cp02, pSecondCamera->PRNrm + 9);

	/////

	InverseMatrix3(pSecondCamera->InvPRNrm, pSecondCamera->PRNrm);
	LinearTransform3D(pSecondCamera->InvPRNrm, pSecondCamera->PRNrm + 9, pSecondCamera->InvPRNrm + 9);

	pSecondCamera->P3DC0Nrm[0] = -pSecondCamera->InvPRNrm[9];
	pSecondCamera->P3DC0Nrm[1] = -pSecondCamera->InvPRNrm[10];
	pSecondCamera->P3DC0Nrm[2] = -pSecondCamera->InvPRNrm[11];

	// transform to the center of projection of the left camera

	pSecondCamera->P3DC0Nrm[0] -= P3DC0Nrm[0];
	pSecondCamera->P3DC0Nrm[1] -= P3DC0Nrm[1];
	pSecondCamera->P3DC0Nrm[2] -= P3DC0Nrm[2];
	//pSecondCamera->P3DC0Nrm[1] = 0.0;
	//pSecondCamera->P3DC0Nrm[2] = 0.0;

	pSecondCamera->InvPRNrm[9]  = -pSecondCamera->P3DC0Nrm[0];
	pSecondCamera->InvPRNrm[10] = -pSecondCamera->P3DC0Nrm[1];
	pSecondCamera->InvPRNrm[11] = -pSecondCamera->P3DC0Nrm[2];
	//pSecondCamera->InvPRNrm[10] = 0.0;
	//pSecondCamera->InvPRNrm[11] = 0.0;
	
	LinearTransform3D(pSecondCamera->PRNrm, pSecondCamera->InvPRNrm + 9, 
		pSecondCamera->PRNrm + 9);

	memset(P3DC0Nrm, 0, 3 * sizeof(double));

	memset(InvPRNrm + 9, 0, 3 * sizeof(double));

	memset(PRNrm + 9, 0, 3 * sizeof(double));
}

void CRVLCamera::DistortionCorrection(double xd, double yd, double &xu, double &yu)
{
	double xd2 = (xd - CenterX) * dpx / sx;
	double yd2 = (yd - CenterY) * dpy;

	double k2 = 1.0 + k * (xd2 * xd2 + yd2 * yd2);
	xu = xd2 * k2 * sx / dpx + CenterX;
	yu = yd2 * k2 / dpy + CenterY;
}


DWORD CRVLCamera::LoadParams(char *FileName)
{
	int i, j;
	FILE *fp = fopen(FileName, "r");

	if(fp == NULL)
		return RVLCAMERA_RETURN_ERR_PARAM_FILE;

	for(i = 0; i < 3; i++)
	{
		for(j = 0; j < 3; j++)
			fscanf(fp, "%lf\t", T + i * 3 + j);

		fscanf(fp, "%lf\n", T + 9 + i);
	}

	fscanf(fp, "Width=%d\n", &Width);
	fscanf(fp, "Height=%d\n", &Height);
	fscanf(fp, "f=%lf\n", &f);
	fscanf(fp, "sx=%lf\n", &sx);
	fscanf(fp, "dpx=%lf\n", &dpx);
	fscanf(fp, "dpy=%lf\n", &dpy);
	fscanf(fp, "CenterX=%lf\n", &CenterX);
	fscanf(fp, "CenterY=%lf\n", &CenterY);
	fscanf(fp, "k=%lf\n", &k);

	fclose(fp);

	GetProjectionMatrix();	

	return RVL_RES_OK;
}


void CRVLCamera::Project3DPoint(double *X,
								double *U,
								int *iU,
								double *CX,
								double *CU)
{
	U[0] = fNrm * X[0] / X[2];
	U[1] = fvNrm * X[1] / X[2];

	RVLU2iU(U, CenterXNrm, CenterYNrm, iU);

	if(CX)
	{
		double J[6];

		RVL3DPointProjectJacobian(X, fNrm, J);

		RVLGetCov2DFromCov3D(CX, J, CU);
	}
}


void CRVLCamera::Project3DPointWithNoise(double *X,
										 double *U,
										 int *iU,
										 double *Unoise)
{
	U[0] = fNrm * X[0] / X[2] + Unoise[0];
	U[1] = fNrm * X[1] / X[2] + Unoise[1];

	RVLU2iU(U, CenterXNrm, CenterYNrm, iU);
}

// Computes homograpy matrix
//
// Details are given in from_disparity_space_to_3D.doc, Section 5: Homography

void CRVLCamera::Homography(double *T,		// Input: transf. matrix:
											//			1. column represents x-axis of the plane c.s.
											//			2. column represents y-axis of the plane c.s.
											//			3. column represents origin of the plane c.s.
											//        all these vectors are represented in the camera c.s.
							double *H)		// Output: homography matrix
{
	MatrixMultiplication(PRNrm, T, H, 3, 3, 3);
}


// Computes homograpy matrix, given plane parameters

void CRVLCamera::Homography(double *N,	double d,	// Input: plane parameters
							double *X,				// Input: x-axis of the plane c.s.
							double *H)				// Output: homography matrix
{
	double Y[3];

	CrossProduct(N, X, Y);

	double T[3 * 3];

	T[0 * 3 + 0] = X[0];
	T[1 * 3 + 0] = X[1];
	T[2 * 3 + 0] = X[2];

	T[0 * 3 + 1] = Y[0];
	T[1 * 3 + 1] = Y[1];
	T[2 * 3 + 1] = Y[2];

	T[0 * 3 + 2] = d * N[0];
	T[1 * 3 + 2] = d * N[1];
	T[2 * 3 + 2] = d * N[2];

	Homography(T, H);
}



BYTE LoadPGM(char *FileName, 
			 PIX_ARRAY *pPixArray)
{
	FILE *fp = fopen(FileName, "rb");

	if(fp == NULL)
		return RVLCAMERA_INPUT_PGM_RES_ERR_NO_FILE;

	unsigned char tmp;

	char line[200];

	fgets(line, 200, fp);

	if(strcmp(line, "P5\xa") == 0)
	{
		fgets(line, 200, fp);

		fgets(line, 200, fp);

		if(sscanf(line, "%d %d\n", &(pPixArray->Width), 
			&(pPixArray->Height)) != 2)
		{
			fclose(fp);

			return RVLCAMERA_INPUT_PGM_RES_ERR_NOT_PGM;
		}

		fgets(line, 200, fp);
	}
	else if(sscanf(line, "P5 %d %d %d\n", &(pPixArray->Width), 
		&(pPixArray->Height), &tmp) != 3)
	{
		fclose(fp);

		return RVLCAMERA_INPUT_PGM_RES_ERR_NOT_PGM;
	}

	if(pPixArray->bOwnData && pPixArray->pPix != NULL)
		delete[] pPixArray->pPix;

	int nPix = pPixArray->Width * pPixArray->Height;

	pPixArray->pPix = new unsigned char[nPix];

	pPixArray->bOwnData = TRUE;
	pPixArray->bColor = FALSE;

	if((int)fread(pPixArray->pPix, 1, nPix, fp) != nPix)
	{
		fclose(fp);

		return RVLCAMERA_INPUT_PGM_RES_ERR_FILE_TOO_SHORT;
	}

	fclose(fp);

	return RVLCAMERA_INPUT_PGM_RES_OK;
}

void SavePGM(char *Filename, 
			 unsigned char *I, 
			 int Width, 
			 int Height,
			 BOOL bFlush)
{
	int i, j;
	FILE *fpbmp;

	fpbmp=fopen(Filename,"wb");

	fprintf(fpbmp, "P5 %d %d 255\n", Width, Height);

	//unsigned char *pI = I + (Height - 1) * Width;
	unsigned char *pI = I;

	for(i = 0; i < Height; i++)
	{
		for(j = 0; j < Width; j++)
			fputc((int)(*(pI++)),fpbmp);

		//pI -= 2 * Width;
	}

	if(bFlush)
		fflush(fpbmp);

	fclose(fpbmp);
}


// J = dU / dX

void RVL3DPointProjectJacobian(double *X, 
							   double f, 
							   double *J)
{
	double k1 = f / X[2];
	double k2 = k1 / X[2];

	J[3*0+0] = k1;
	J[3*0+1] = 0.0;
	J[3*0+2] = -k2 * X[0];

	J[3*1+0] = 0.0;
	J[3*1+1] = k1;
	J[3*1+2] = -k2 * X[1];
}

void CRVLCamera::StereoRecon(int *iU, 
							 short int d, 
							 double b, 
							 double dCenterXNrm,
							 double *U,
							 double &fd,
							 double *X)
{
	fd = (double)d / 16.0 - dCenterXNrm;
	X[2] = fNrm * b / fd;
	RVLiU2U(iU, CenterXNrm, CenterYNrm, U);
	X[0] = X[2] / fNrm * U[0];
	X[1] = X[2] / fNrm * U[1];
}

void CRVLCamera::StereoRecon(double *U, 
							 double d, 
							 double b, 
							 double *X)
{
	X[2] = fNrm * b / d;
	X[0] = X[2] / fNrm * U[0];
	X[1] = X[2] / fNrm * U[1];
}

void CRVLCamera::StereoProject3DPointWithNoise(double *X,
											   double b,
											   double dCenterXNrm,
											   double *U,
											   int *iU,
											   double &fd,
											   short int &d,
											   double *Unoise)
{
	Project3DPointWithNoise(X, U, iU, Unoise);

	//fd = fNrm * b / X[2] + 2.0 * Unoise[2];

	fd = fNrm * b / X[2] + Unoise[2];

	d = (short int)(DOUBLE2INT(fd * 16.0));	
}

void CRVLCamera::StereoProject3DPoint(double *X,
									  double b, 
									  double *U, 
									  double &d)
{
	U[0] = fNrm * X[0] / X[2];
	U[1] = fNrm * X[1] / X[2];
	d = fNrm * b / X[2];
}


void CRVLCamera::StereoReconWithUncert(int *iU, 
									   short int d, 
									   double b, 
									   double dCenterXNrm,
									   int du,
									   double *X,
									   double *V,
									   double &l)
{
	int iU2[2];
	double X1[3], X2[3];
	double U[2];
	double fd;

	iU2[0] = iU[0] + du;
	iU2[1] = iU[1];

	StereoRecon(iU2, d - 16 * du, b, dCenterXNrm, U, fd, X1);
	
	iU2[0] = iU[0] - du;
	iU2[1] = iU[1];

	StereoRecon(iU2, d + 16 * du, b, dCenterXNrm, U, fd, X2);

	X[0] = 0.5 * (X1[0] + X2[0]);
	X[1] = 0.5 * (X1[1] + X2[1]);
	X[2] = 0.5 * (X1[2] + X2[2]);

	double W[3];

	RVLDif3D(X2, X, W);

	l = sqrt(RVLDotProduct(W, W));

	V[0] = W[0] / l;
	V[1] = W[1] / l;
	V[2] = W[2] / l;	
}

void CRVLCamera::StereoReconLineUncert(double *U, 
									   double d,									   
									   double b,
									   double stdd,
									   double *V,
									   double &l)
{
	double U2[2];
	double X1[3], X2[3];

	U2[0] = U[0] + stdd;
	U2[1] = U[1];

	StereoRecon(U2, d + 2.0 * stdd, b, X1);
	
	U2[0] = U[0] - stdd;
	U2[1] = U[1];

	StereoRecon(U2, d - 2.0 * stdd, b, X2);

	double W[3];

	RVLDif3D(X2, X1, W);

	l = sqrt(RVLDotProduct(W, W));

	V[0] = W[0] / l;
	V[1] = W[1] / l;
	V[2] = W[2] / l;	

	l *= 0.5;
}

void CRVLCamera::StereoReconWithUncert(int *iU, 
									   short int d, 
									   double b, 
									   double dCenterXNrm,
									   double varU,
									   double *X,
									   double *C)
{
	double U[2];
	double fd;

	StereoRecon(iU, d, b, dCenterXNrm, U, fd, X);

	double d2 = fd * fd;
	double k = b / d2;
	k *= (k * varU);
	double u2 = U[0] - fd;
	double sumu = U[0] + u2;
	double v2 = 2.0 * U[1];
	double ev = 1.0;

	C[0*3+0]            = k * (U[0] * U[0] + u2 * u2);
	C[0*3+1] = C[1*3+0] = k * U[1] * sumu;
	C[0*3+2] = C[2*3+0] = k * fNrm * sumu;
	C[1*3+1]            = k * (v2 * U[1] + d2 * ev * ev);
	C[1*3+2] = C[2*3+1] = k * v2 * fNrm;
	C[2*3+2]            = k * 2.0 * fNrm * fNrm;	
}

void CRVLCamera::StereoReconWithUncert(double *U, 
									   double d, 
									   double b, 
									   double dCenterXNrm,
									   double varU,
									   double *X,
									   double *C)
{
	StereoRecon(U, d, b, X);

	double d2 = d * d;
	double k = b / d2;
	k *= (k * varU);
	double u2 = U[0] - d;
	double sumu = U[0] + u2;
	double v2 = 2.0 * U[1];
	double ev = 1.0;

	C[0*3+0]            = k * (U[0] * U[0] + u2 * u2);
	C[0*3+1] = C[1*3+0] = k * U[1] * sumu;
	C[0*3+2] = C[2*3+0] = k * fNrm * sumu;
	C[1*3+1]            = k * (v2 * U[1] + d2 * ev * ev);
	C[1*3+2] = C[2*3+1] = k * v2 * fNrm;
	C[2*3+2]            = k * 2.0 * fNrm * fNrm;	
}

void CRVLCamera::StereoReconWithUncert2(double *U, 
									   double d, 
									   double b,
									   double varU,
									   double vard, 
									   double *X,
									   double *C)
{
	StereoRecon(U, d, b, X);

	double u = U[0];
	double v = U[1];
	double k1 = b / d;
	k1 *= k1;
	double k2 = k1 / (d * d) * vard;
	k1 *= varU;

	C[0*3+0] =            k1 + k2*u*u;
	C[0*3+1] = C[1*3+0] = k2*u*v;
	C[0*3+2] = C[2*3+0] = k2*fNrm*u;
	C[1*3+1] =            k1 + k2*v*v;
	C[1*3+2] = C[2*3+1] = k2*fNrm*v;
	C[2*3+2] =            k2*fNrm*fNrm;
}

void CRVLCamera::KinectReconWithUncert(double u,
									   double v,
									   double d, 
									   double d0,
									   double k,
									   double Uc,
									   double Vc,
									   double Fu,
									   double Fv,
									   double varU,
									   double vard, 
									   double *C)
{
	double diffu = u - Uc;
	double diffv = v - Vc;
	double diffd = d - d0;
	double diffd2 = diffd * diffd;

	double konst = k * k * vard / (diffd2 * diffd2);

	

	C[0*3+0] =            konst * ((varU * diffd2) + (vard * diffu * diffu)) / (vard * Fu * Fu);
	C[0*3+1] = C[1*3+0] = konst * diffu * diffv / (Fu * Fv);
	C[0*3+2] = C[2*3+0] = konst * diffu / Fu;
	C[1*3+1] =            konst * ((varU * diffd2) + (vard * diffv * diffv)) / (vard * Fv * Fv);
	C[1*3+2] = C[2*3+1] = konst * diffv / Fv;
	C[2*3+2] =            konst;
}


void EvenLineInterpolation(PIX_ARRAY *pPixArray)
{
	int Width = pPixArray->Width;
	int Height = pPixArray->Height;

	unsigned char *pPixSrc1 = pPixArray->pPix;
	unsigned char *pPixSrc2 = pPixSrc1 + 2 * Width;
	unsigned char *pPixTgt = pPixSrc1 + Width;
	unsigned char *pPixEnd = pPixSrc1 + Height * Width;

	unsigned char *pRowEnd;

	for (; pPixSrc2 < pPixEnd; pPixSrc1 += Width, pPixSrc2 += Width, pPixTgt += Width)
	{
		pRowEnd = pPixTgt;

		for(; pPixSrc1 < pRowEnd; pPixSrc1++, pPixSrc2++, pPixTgt++)
			*pPixTgt = (unsigned char)(((int)(*pPixSrc1) + (int)(*pPixSrc2)) >> 1);
	}

	memcpy(pPixTgt, pPixSrc1, Width * sizeof(unsigned char));
}



void Stretch(PIX_ARRAY *pPixArray)
{
	int u, v;

	unsigned char *pPix = new unsigned char[pPixArray->Height * pPixArray->Width * 2];
	unsigned char *pPixSrc = pPixArray->pPix;
	unsigned char *pPixSrc2 = pPixSrc + pPixArray->Width;
	unsigned char *pPixTgt = pPix;

	for (v = 0; v < pPixArray->Height - 1; v++)
	{
		memcpy(pPixTgt, pPixSrc, pPixArray->Width * sizeof(unsigned char));

		pPixTgt += pPixArray->Width;

		for(u = 0; u < pPixArray->Width; u++)
			*(pPixTgt++) = (unsigned char)(((int)(*(pPixSrc++)) + (int)(*(pPixSrc2++))) >> 1);
	}

	memcpy(pPixTgt, pPixSrc, pPixArray->Width * sizeof(unsigned char));

	pPixTgt += pPixArray->Width;

	memcpy(pPixTgt, pPixSrc, pPixArray->Width * sizeof(unsigned char));

	pPixArray->Height *= 2;

	delete[] pPixArray->pPix;

	pPixArray->pPix = pPix;
}

void RVLStereoProject3DPoint(double *X,
							 double f,
							 double b, 
							 double *U, 
							 double &d)
{
	double k = f / X[2];
	U[0] = k * X[0];
	U[1] = k * X[1];
	d = k * b;
}


void RVLHomography(double *R, 
				   double *t, 
				   double fu,
				   double fv,
				   double uc,
				   double vc,
				   double *H)
{
	RVLMXEL(H, 3, 0, 0) = fu * RVLMXEL(R, 3, 0, 0) + uc * RVLMXEL(R, 3, 2, 0);
	RVLMXEL(H, 3, 0, 1) = fu * RVLMXEL(R, 3, 0, 1) + uc * RVLMXEL(R, 3, 2, 1);
	RVLMXEL(H, 3, 0, 2) = fu * t[0] + uc * t[2];
	RVLMXEL(H, 3, 1, 0) = fv * RVLMXEL(R, 3, 1, 0) + vc * RVLMXEL(R, 3, 2, 0);
	RVLMXEL(H, 3, 1, 1) = fv * RVLMXEL(R, 3, 1, 1) + vc * RVLMXEL(R, 3, 2, 1);
	RVLMXEL(H, 3, 1, 2) = fv * t[1] + vc * t[2];
	RVLMXEL(H, 3, 2, 0) = RVLMXEL(R, 3, 2, 0);
	RVLMXEL(H, 3, 2, 1) = RVLMXEL(R, 3, 2, 1);
	RVLMXEL(H, 3, 2, 2) = t[2];
}

void Projection(double *P, double *PM, int &u, int &v)
{
	double fx = PM[0 * 3 + 0] * P[0] + PM[0 * 3 + 1] * P[1] + PM[0 * 3 + 2] * P[2] + PM[3 * 3 + 0];
	double fy = PM[1 * 3 + 0] * P[0] + PM[1 * 3 + 1] * P[1] + PM[1 * 3 + 2] * P[2] + PM[3 * 3 + 1];
	double fz = PM[2 * 3 + 0] * P[0] + PM[2 * 3 + 1] * P[1] + PM[2 * 3 + 2] * P[2] + PM[3 * 3 + 2];

	u = DOUBLE2INT(fx / fz);
	v = DOUBLE2INT(fy / fz);
}

void Projection(double *P, double *PM, double &u, double &v)
{
	double fx = PM[0 * 3 + 0] * P[0] + PM[0 * 3 + 1] * P[1] + PM[0 * 3 + 2] * P[2] + PM[3 * 3 + 0];
	double fy = PM[1 * 3 + 0] * P[0] + PM[1 * 3 + 1] * P[1] + PM[1 * 3 + 2] * P[2] + PM[3 * 3 + 1];
	double fz = PM[2 * 3 + 0] * P[0] + PM[2 * 3 + 1] * P[1] + PM[2 * 3 + 2] * P[2] + PM[3 * 3 + 2];

	u = fx / fz;
	v = fy / fz;
}

void GetKMatrix(double *K, double f, double sx, double dpx, double dpy, double Cx, double Cy)
{
	K[0 * 3 + 0] = sx * f / dpx;
	K[0 * 3 + 1] = 0.0;
	K[0 * 3 + 2] = Cx;
	K[1 * 3 + 0] = 0.0;
	K[1 * 3 + 1] = f / dpy;
	K[1 * 3 + 2] = Cy;
	K[2 * 3 + 0] = 0.0;
	K[2 * 3 + 1] = 0.0;
	K[2 * 3 + 2] = 1.0;
}
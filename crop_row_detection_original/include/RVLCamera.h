// RVLCamera.h: interface for the CRVLCamera class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLCAMERA_H__1734A925_42F4_4559_B22D_6436B8064D7D__INCLUDED_)
#define AFX_RVLCAMERA_H__1734A925_42F4_4559_B22D_6436B8064D7D__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define RVLCAMERA_RETURN_ERR_PARAM_FILE				0x00000001
#define RVLCAMERA_RETURN_ERR_FALSE_IMAGE_FORMAT		0x00000002
#define RVLCAMERA_INPUT_PGM_RES_OK					0
#define RVLCAMERA_INPUT_PGM_RES_ERR_NO_FILE			1
#define RVLCAMERA_INPUT_PGM_RES_ERR_NOT_PGM			2
#define RVLCAMERA_INPUT_PGM_RES_ERR_FILE_TOO_SHORT	3
#define RVLCAMERA_FLAG_FV							0x00000001

BYTE LoadPGM(char *FileName, 
	PIX_ARRAY *pPixArray);
void SavePGM(char *Filename, 
	unsigned char *I, 
	int Width, 
	int Height,
	BOOL bFlush = FALSE);
void RVL3DPointProjectJacobian(double *X, 
							   double f, 
							   double *J);
void EvenLineInterpolation(PIX_ARRAY *pPixArray);
void Stretch(PIX_ARRAY *pPixArray);
void RVLStereoProject3DPoint(double *X,
							 double f,
							 double b, 
							 double *U, 
							 double &d);
void RVLHomography(double *R, 
				   double *t, 
				   double fu,
				   double fv,
				   double uc,
				   double vc,
				   double *H);
void Projection(double *P, double *PM, int &u, int &v);
void Projection(double *P, double *PM, double &u, double &v);
void GetKMatrix(double *K, double f, double sx, double dpx, double dpy, double Cx, double Cy);

class CRVLCamera  
{
public:
	DWORD m_Flags;
	int Width, Height;
	double f;
	double k;
	double dpx, dpy, sx;
	double CenterX, CenterY;
	double T[12];
	double PR[12];
	double InvPR[12];
	double fNrm, fvNrm;
	double CenterXNrm, CenterYNrm;
	double PRNrm[12];
	double InvPRNrm[12];
	double NrmTransf[9];
	double TNrm[12];
	double TiltAngle;
	double P3DC0Nrm[3];
	short int *m_nrmud;
	short int *m_nrmvd;
	PIX_ARRAY m_Image;
	PIX_ARRAY m_nrmImage;
	IplImage *m_pRGBImage;

public:	
	void Homography(double *N,	double d,	
					double *X,		
					double *H);
	void Homography(double *T, double *H);
	void StereoReconLineUncert(double *U, 
							   double d, 
							   double b, 
							   double stdd,
							   double *V,
							   double &l);
	void StereoProject3DPoint(double *X, 
							  double b, 
							  double *U, 
							  double &d);
	void StereoReconWithUncert2(double *U, 
								double d, 
								double b, 
								double varU,
								double vard,
								double *X,
								double *C);
	void StereoReconWithUncert(int *iU, 
							   short int d, 
							   double b, 
							   double dCenterXNrm,
							   double varU,
							   double *X,
							   double *C);
	void StereoReconWithUncert(double *U, 
							   double d, 
							   double b, 
							   double dCenterXNrm,
							   double varU,
							   double *X,
							   double *C);
	void StereoReconWithUncert(int *iU, 
							   short int d, 
							   double b, 
							   double dCenterXNrm,
							   int du,
							   double *X,
							   double *V,
							   double &l);
	void StereoProject3DPointWithNoise(double *X,
									   double b,
									   double dCenterXNrm,
									   double *U,
									   int *iU,
									   double &fd,
									   short int &d,
									   double *Unoise);
	void StereoRecon(int *iU, 
					 short int d, 
					 double b, 
					 double dCenterXNrm,					 
					 double *U,
					 double &fd,
					 double *X);
	void StereoRecon(double *U, 
					 double d, 
					 double b, 
					 double *X);
	void Project3DPoint(double *X,
					    double *U,
					    int *iU,
					    double *CX = NULL,
					    double *CU = NULL);
	void Project3DPointWithNoise(double *X,
								 double *U,
								 int *iU,
								 double *Unoise);
	void DistortionCorrection(double xd, double yd, double &xu, double &yu);
	void GetParallelStereoCameraSystem(CRVLCamera *pSecondCamera);
	DWORD SetImage(PIX_ARRAY *pImage);
	void NormalizeImage();
	void Distortion(double xu, double yu, double &xd, double &yd);
	void CreateImageNrmLookupTable(int uShift);
	void GetProjectionMatrix();
	DWORD LoadParams(char *FileName);
	DWORD Init(char *CameraParamsFileName);
	void KinectReconWithUncert( double u,
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
								double *C);
	CRVLCamera();
	virtual ~CRVLCamera();

	BOOL IsInsideImage(int *iU)
	{
		return ((iU[0] >> 1) >= 0 && (iU[0] >> 1) < Width && 
			(iU[1] >> 1) >= 0 && (iU[1] >> 1) < Height);
	}
};

inline void RVLiU2U(int *iU, double CenterXNrm, double CenterYNrm, double *U)
{
	U[0] = (double)(iU[0] >> 1) - CenterXNrm;
	U[1] = (double)(iU[1] >> 1) - CenterYNrm;
};

inline void RVLU2iU(double *U, double CenterXNrm, double CenterYNrm, int *iU)
{
	iU[0] = (DOUBLE2INT(U[0] + CenterXNrm) << 1) + 1;
	iU[1] = (DOUBLE2INT(U[1] + CenterYNrm) << 1) + 1;
};

inline void RVLiU2U(int *iU, double *U)
{
	U[0] = (double)(iU[0] >> 1);
	U[1] = (double)(iU[1] >> 1);
};

inline void RVLU2iU(double *U, int *iU)
{
	iU[0] = (DOUBLE2INT(U[0]) << 1) + 1;
	iU[1] = (DOUBLE2INT(U[1]) << 1) + 1;
};

#endif // !defined(AFX_RVLCAMERA_H__1734A925_42F4_4559_B22D_6436B8064D7D__INCLUDED_)

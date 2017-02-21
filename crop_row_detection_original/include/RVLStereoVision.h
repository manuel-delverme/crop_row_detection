// RVLStereoVision.h: interface for the CRVLStereoVision class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLSTEREOVISION_H__B3E4626B_A9A2_4C4F_ABA1_A0999DBE752E__INCLUDED_)
#define AFX_RVLSTEREOVISION_H__B3E4626B_A9A2_4C4F_ABA1_A0999DBE752E__INCLUDED_

// Added by ClassView
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define RVLSTEREO_FLAGS_CALIB_METHOD_SVS		0x00000001
#define RVLSTEREO_FLAGS_METHOD					0x0000000e
#define RVLSTEREO_FLAGS_METHOD_SVS				0x00000002
#define RVLSTEREO_FLAGS_METHOD_OPENCV			0x00000004
#define RVLSTEREO_FLAGS_METHOD_KINECT			0x00000008
#define RVLSTEREO_FLAGS_EVEN_LINE_INTERPOLATION	0x00000020
#define RVLSTEREO_FLAGS_CONTOUR_RECONSTRUCTION	0x00000040
#define RVLSTEREO_FLAGS_DISPARITY_OFFSET		0x00000180
#define RVLSTEREO_FLAGS_DISPARITY_OFFSET_FILE	0x00000080
#define RVLSTEREO_FLAGS_DISPARITY_OFFSET_AUTO	0x00000100
#define RVLSTEREO_STEREO_POINT_ARRAY			0x00000004
#define RVLSTEREO_RES_CANT_OPEN_CAMERA			0x0001
#define RVLSTEREO_RES_CANT_LOAD_PARAMS			0x0002
#define RVLSTEREO_RES_CANT_SET_FRAME_RATE		0x0003
#define RVLSTEREO_RES_CANT_START_ACQUISITION	0x0004
#define RVLSTEREO_RES_IMAGE_ACQUISITION_FAILED	0x0005

#define RVL3DPOINT_FLAG_3D		0x01
#define RVL3DPOINT_FLAG_W		0x02
#define RVL3DPOINT_FLAG_MASK	0x04

struct RVLDISPARITYMAP
{
	short *Disparity;
	int Width, Height;
	int u0, v0;
	int DisparityOffset;
};

struct RVLSTEREOPOINT
{
	short disparity;
	int u, v, s;
};

struct RVL3DPOINT
{
	double X[3];
	double Wy;
	//double C3D[6];
	BYTE Flags;
	int u, v, d;
};

struct RVLTRIANGULATION_PARAMS
{
	double w1, w3, f1, f2;
};

struct RVLKINECT_PARAMS
{
	double k;
	double d0;
	double depthFu;
	double depthFv;
	double depthUc;
	double depthVc;
	double depthFu0;
	double depthFv0;
	double depthUc0;
	double depthVc0;
	double rgbFu;
	double rgbFv;
	double rgbUc;
	double rgbVc;
	double *pZProjLT;
	//CRVL3DPose *pPose;	
};

/*void RVLDisplayDisparityMap(RVLDISPARITYMAP *pDisparityMap, 
							BOOL bInverse, 
							PIX_ARRAY &DisparityPixArray);*/
/*void RVLDisplayDisparityMapColor(RVLDISPARITYMAP *pDisparityMap,
								 short int nDisp,
								 BOOL bInverse, 
								 PIX_ARRAY *pDisparityPixArray,
								 unsigned int Format = RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY);*/
/*void RVLDisplayDisparityMapColor(RVLDISPARITYMAP *pDisparityMap,
								 short int nDisp,
								 BOOL bInverse, 
								 IplImage *pDisparityImage,
								 unsigned int Format = RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY);*/
/*BOOL RVLImportDisparityImage(char *FileName, 
	 					     RVLDISPARITYMAP *pDisparityImage,
							 short *zToDepthLookupTable = NULL);
void RVLSaveStereoPts(FILE *fp,
					  RVL3DPOINT2 *PtArray,
					  int nPts);
void RVLGetKinect3DData(int *pUVDPt, double *pXYZPt, RVLKINECT_PARAMS kinectParams);

void RVLGetKinect2DData(int *pUVDPt, double *pXYZPt, RVLKINECT_PARAMS kinectParams);

void RVLGetKinectRGBProjection(int *pUVDPt, int *pRGBPt, RVLKINECT_PARAMS *kinectParams);

void RVLGetKinectRGBProjection(double *pXYZPt, int *pRGBPt, RVLKINECT_PARAMS *kinectParams);

void RVLSaveDepthImage(short *iDepth, int w, int h, char *DepthFileName, 
					   DWORD SrcFormat = RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY,
					   DWORD TgtFormat = RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY, 
					   short *zToDepthLT = NULL);*/

class CRVLStereoVision  
{
public: 
	RVLSTEREOPOINT *m_StereoPointArray;
	int m_nStereoPoints;
	DWORD m_Flags;
	CRVLMem *m_pMem, *m_pMem2;
	int m_TolDisparity;
	RVLDISPARITYMAP m_DisparityMap;
	CRVLCamera *m_pCameraL, *m_pCameraR;
	RVLTRIANGULATION_PARAMS *m_TriangulationLookupTable;
	double m_k1, m_k3, m_k4;
	double m_BaseLenNrm;
	double m_varIP;
	int m_ImageWidth, m_ImageHeight;
	int m_u1Shift;
	RVL3DPOINT *m_3DMap;
	int m_EdgeMagnitudeThr;
	short m_minDisparity;
	double m_UncertCoeff, m_WDisparity;
	double m_maxz;
	double m_3DMapRotCorr[9];
	void *m_vpVideoObject;
	void *m_vpImageAcquisition;
	void *m_vpStereoImage;
	double m_minDist;
	int m_nDisp;
	int m_nSubPixDisps;
	int m_WinSize;				// size of window for area-based stereo correlation
	int m_ConfThr;				// confidence threshold for area-based stereo correlation
	int m_UniquenessThr;
	int m_DisparityOffset;		
	char *m_DisparityFileName;
	char *m_ParamFileName;
	int m_DisparityOffsetNrm;
	void *m_vpEdgeDetectL, *m_vpEdgeDetectR;
	unsigned short *m_SADArray;
	CRVLParameterList m_ParamList;
	RVLKINECT_PARAMS m_KinectParams;
	int m_KinectScale;
private:
	IplImage *pDisparityImage;

public:
	void GetKinectProjectionMatrix(double *P);
	void CreateParamList(CRVLMem *pMem);
	void SADStatistics();
	void Statistics(float *dGuI, 
					float *dGvI);
	void SetDisparityOffset(int DisparityOffset);
	void RVLImportDisparityImage();
	void CreateStereoPointArray();
	void GetDisparityOffset();
	void InitCamerasSVS(char *ParamFileName);
	void CreateDisparityMap();
	void GetUVDPlane(	double *N,
						double d,
						double &a,
						double &b,
						double &c);

#ifdef SVS
	void CameraStop();
	WORD ImageAcquisition(int n = 1);
	WORD CameraInit();
	
	void NormalizeStereoImage(PIX_ARRAY *pImageL, PIX_ARRAY *pImageR, 
							  PIX_ARRAY *pNrmImageL, PIX_ARRAY *pNrmImageR);
#endif
	void Get3DKinect(int u, int v, int d, double *X);
	void DisplayHeightMap(PIX_ARRAY &HeightPixArray, 
		  				  double y0);
	//void Create3DMap(CRVL3DPose *pPoseLA);
	void GetMinDisparity();
	/*void Get3DWithUncert(unsigned char *EdgeMagnitude, 
		CRVL3DPose *pPoseLA);*/
	void CreateTriangulationLookupTable();
	void Init();
	void GetTriangulationUncert(double *X, double *CX);
	void SetDisparityMap(RVLDISPARITYMAP *pDisparityMap);
	CRVLStereoVision();
	virtual ~CRVLStereoVision();
	// the following two functions should be transfered to CRVL3DLine2
	//BOOL Get3DLine(CRVL2DLine2 *p2DLine, 
	//	RVLSTEREOPOINT *StereoPoint, 
	//	RVLSTEREOPOINT *StereoPointBuffer,
	//	CRVLC3D *pC3DLine);
	//void Get3DLines(CRVLMPtrChain *p2DLineArray, 
	//				CRVLC3D *pC3DLine);

	inline void Triangulation4(int u, int v, short disparity, double *P3D, int &u2_16, double &den,
		RVLTRIANGULATION_PARAMS *pTriangulationParams)
	{
		u2_16 = (u << 4) - ((int)disparity + m_DisparityOffsetNrm);

		double fu2 = (double)u2_16;

		den = pTriangulationParams->w3 * fu2 - pTriangulationParams->w1;

		double z = -(m_k3 * fu2 - m_k1) / den;

		P3D[0] = pTriangulationParams->f1 * z;
		P3D[1] = pTriangulationParams->f2 * z;
		P3D[2] = z;
	}

	inline void Triangulation4(int u, int v, short disparity,
		double *P3D, int &u2_16, double &den, RVLTRIANGULATION_PARAMS **ppTriangulationParams)
	{
		*ppTriangulationParams = m_TriangulationLookupTable +
			m_ImageWidth * v + u;

		Triangulation4(u, v, disparity, P3D, u2_16, den, *ppTriangulationParams);
	}

	inline void Triangulation4(int u, int v, double *P3D, int &u2_16, double &den,
		RVLTRIANGULATION_PARAMS **ppTriangulationParams)
	{
		short disparity = m_DisparityMap.Disparity[(v - m_DisparityMap.v0) * 
			m_DisparityMap.Width + (u - m_DisparityMap.u0)];

		Triangulation4(u, v, disparity, P3D, u2_16, den, ppTriangulationParams);
	}

	inline void GetTriangulationUncert4(double den,
												RVLTRIANGULATION_PARAMS *pTriangulationParams,
												double *W3D1)
	{
		double zUncert = (m_k3 * pTriangulationParams->w1 - m_k1 * pTriangulationParams->w3) /
			(den * den) * m_WDisparity;

		W3D1[0] = pTriangulationParams->f1 * zUncert;
		W3D1[1] = pTriangulationParams->f2 * zUncert;
		W3D1[2] = zUncert;
	}
};

#endif // !defined(AFX_RVLSTEREOVISION_H__B3E4626B_A9A2_4C4F_ABA1_A0999DBE752E__INCLUDED_)

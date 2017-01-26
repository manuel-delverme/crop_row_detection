// RVLStereoVision.cpp: implementation of the CRVLStereoVision class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "Platform.h"

#ifdef SVS
#include "svs.h"
#include "svsclass.h"
#endif

#include <highgui.h>
#include "RVLCore.h"

#ifdef RVLSTEREO_CORRESPONDENCE_SAD_ARRAY
CV_IMPL void cvFindStereoCorrespondenceBM_RVL( const CvArr* leftarr, const CvArr* rightarr,
                              CvArr* disparr, CvStereoBMState* state, unsigned short *SADArray );
#endif

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLStereoVision::CRVLStereoVision()
{
	m_Flags = RVLSTEREO_FLAGS_METHOD_KINECT;
	m_TolDisparity = 16;		// pix
	m_varIP = 0.333 * 0.333;	// pix^2
	m_TriangulationLookupTable = NULL;
	m_u1Shift = 0;
	m_EdgeMagnitudeThr = 255 - 4;
	m_minDisparity = 0;			// pix
	m_UncertCoeff = 1.0;
	m_WDisparity = m_UncertCoeff * 16.0;
	m_maxz = 5000.0;			// mm
	m_3DMap = NULL;
	m_nDisp = 128;				// pix
	m_nSubPixDisps = 8;
	m_WinSize = 11;				// pix
	m_ConfThr = 16;
	m_UniquenessThr = 4;
	m_vpVideoObject = NULL;	
	m_vpEdgeDetectR = NULL;
	m_SADArray = NULL;
	m_KinectParams.depthFu0 = 584.70194597700402;
	m_KinectParams.depthUc0 = 318.55964537649561;
	m_KinectParams.depthFv0 = 585.70332900816618;
	m_KinectParams.depthVc0 = 256.14501544470505;
	
#ifdef SVS
	m_vpImageAcquisition = (void *)(new svsStoredImages);
	m_vpStereoImage = (void *)(new svsStereoImage);
#else
	m_vpImageAcquisition = NULL;
	m_vpStereoImage = NULL;
#endif
	m_DisparityOffset = 0;
	m_DisparityOffsetNrm = (m_DisparityOffset << 4);

	m_DisparityFileName = NULL;
	m_ParamFileName = NULL;

	m_DisparityMap.Width = m_DisparityMap.Height = 0;
	m_DisparityMap.Disparity = NULL;

	/*m_KinectParams.pZProjLT = NULL;
	m_KinectParams.pPose = NULL;
	m_KinectScale = 2;*/
}

CRVLStereoVision::~CRVLStereoVision()
{
	if(m_TriangulationLookupTable != NULL)
		delete[] m_TriangulationLookupTable;

	if(m_3DMap != NULL)
		delete[] m_3DMap;

#ifdef SVS
	delete (svsStoredImages *)m_vpImageAcquisition;

	if(m_vpVideoObject)
		CameraStop();
	else
		delete (svsStereoImage *)m_vpStereoImage;
#else
	if(m_DisparityMap.Disparity)
		delete[] m_DisparityMap.Disparity;
#endif

	if(m_ParamFileName)
		delete[] m_ParamFileName;

	if(m_SADArray)
		delete[] m_SADArray;

	/*if(m_KinectParams.pZProjLT)
		delete[] m_KinectParams.pZProjLT;

	if(m_KinectParams.pPose)
		delete m_KinectParams.pPose;*/

}




void CRVLStereoVision::SetDisparityMap(RVLDISPARITYMAP *pDisparityMap)
{
	m_DisparityMap = *pDisparityMap;

	m_DisparityOffsetNrm = (m_DisparityMap.DisparityOffset << 4);
}



void CRVLStereoVision::GetTriangulationUncert(double *X, double *CX)
{
	double u1, v1, u2, v2;

	Projection(X, m_pCameraL->PRNrm, u1, v1);
	Projection(X, m_pCameraR->PRNrm, u2, v2);

	u1 -= m_pCameraL->CenterXNrm;
	v1 -= m_pCameraL->CenterYNrm;
	u2 -= m_pCameraR->CenterXNrm;

	// u1 / f = x / z;   u2 / f = (x - b) / z  =>
	// =>   z = f * b / (u1 - u2);   x = u1 * z / f;   y = v1 * z / f
	// J = jacobian([x y z]', [u1 v1 u2 v2]')
	// P = J'* J

	double d = u1 - u2;
	double d2 = d * d;
	double k = m_BaseLenNrm / d2;
	k *= k;
	double sumu = u1 + u2;
	v2 = 2.0 * v1;
	double ev = 1.0;

	CX[0] = k * (u1 * u1 + u2 * u2) * m_varIP;
	CX[1] = CX[3] = k * v1 * sumu * m_varIP;
	CX[2] = CX[6] = k * m_pCameraL->fNrm * sumu * m_varIP;
	CX[4] = k * (v2 * v1 + d2 * ev * ev) * m_varIP;
	CX[5] = CX[7] = k * v2 * m_pCameraL->fNrm * m_varIP;
	CX[8] = k * 2.0 * m_pCameraL->fNrm * m_pCameraL->fNrm * m_varIP;
}

void CRVLStereoVision::Init()
{
	if(m_Flags & RVLSTEREO_FLAGS_CALIB_METHOD_SVS)
		m_BaseLenNrm = m_pCameraR->InvPRNrm[9] - m_pCameraL->InvPRNrm[9];
	
	else
	{
		int i;

		m_BaseLenNrm = 0.0;

		double dP3DC0Nrm;

		for(i = 0; i < 3; i++)
		{
			dP3DC0Nrm = m_pCameraL->P3DC0Nrm[i] - m_pCameraL->P3DC0Nrm[i];

			m_BaseLenNrm += (dP3DC0Nrm * dP3DC0Nrm);
		}

		m_BaseLenNrm = sqrt(m_BaseLenNrm);
	}

	m_pCameraL->m_nrmImage.Width = m_pCameraL->m_Image.Width = m_pCameraL->Width;
	m_pCameraL->m_nrmImage.Height = m_pCameraL->m_Image.Height = m_pCameraL->Height;

	if(m_Flags & RVLSTEREO_FLAGS_METHOD_KINECT)
	{
		//Initialize KINECT params
		//KINECT constants (divided by 2 because of subsampled resolution of 320x240)
		
		//OLD KINECT DATA!
		//m_KinectParams.depthFu = 582.26972991070841/2;
		//m_KinectParams.depthUc = 333.78575299556587/2;
		//m_KinectParams.depthFv = 562.98284310455313/2;
		//m_KinectParams.depthVc = 238.07133397745864/2;

		//m_KinectParams.rgbFu = 521.14532308239393/2;
		//m_KinectParams.rgbUc = 311.66743557620077/2;
		//m_KinectParams.rgbFv = 516.65646258458503/2;
		//m_KinectParams.rgbVc = 259.25911293440481/2;
		
		//m_KinectParams.pPose = new CRVL3DPose;
		//m_KinectParams.pPose->m_Rot[0] = 0.99899332935846452;
		//m_KinectParams.pPose->m_Rot[1] = -0.0015284464595360591;
		//m_KinectParams.pPose->m_Rot[2] = 0.044832931520376568;
		//m_KinectParams.pPose->m_Rot[3] = 0.0027173126649378660;
		//m_KinectParams.pPose->m_Rot[4] = 0.99964594791272010;
		//m_KinectParams.pPose->m_Rot[5] = -0.026468755799252012;
		//m_KinectParams.pPose->m_Rot[6] = -0.044776602251303213;
		//m_KinectParams.pPose->m_Rot[7] = 0.026563935572497543;
		//m_KinectParams.pPose->m_Rot[8] = 0.99864378695194855;
		
		//m_KinectParams.pPose->m_X[0] =  0.026709940276080747;
		//m_KinectParams.pPose->m_X[1] = -0.0056095917183017910; 
		//m_KinectParams.pPose->m_X[2] = -0.0070003194334035236;

		/////////// KORISTENI PARAMETRI ///////////

		/*m_KinectParams.depthFu = m_KinectParams.depthFu0 / m_KinectScale;
		m_KinectParams.depthUc = m_KinectParams.depthUc0 / m_KinectScale;
		m_KinectParams.depthFv = m_KinectParams.depthFv0 / m_KinectScale;
		m_KinectParams.depthVc = m_KinectParams.depthVc0 / m_KinectScale;
		
		m_KinectParams.rgbFu = 521.50932707979371/m_KinectScale;
		m_KinectParams.rgbUc = 322.17660535484873/m_KinectScale;
		m_KinectParams.rgbFv = 522.56240592356971/m_KinectScale;
		m_KinectParams.rgbVc = 263.39316821976075/m_KinectScale;
		
		m_KinectParams.pPose = new CRVL3DPose;
		m_KinectParams.pPose->m_Rot[0] = 0.99998448339672230;
		m_KinectParams.pPose->m_Rot[1] = -0.0026635069214754063;
		m_KinectParams.pPose->m_Rot[2] = -0.0048927187400962229;
		m_KinectParams.pPose->m_Rot[3] = 0.0027115637569482538;
		m_KinectParams.pPose->m_Rot[4] = 0.99994789104620374;
		m_KinectParams.pPose->m_Rot[5] = 0.0098418806252346565;
		m_KinectParams.pPose->m_Rot[6] = 0.0048662498684758099;
		m_KinectParams.pPose->m_Rot[7] = -0.0098549948314860733;
		m_KinectParams.pPose->m_Rot[8] = 0.99993959752031469;
		
		m_KinectParams.pPose->m_X[0] =  0.026546242279625421;
		m_KinectParams.pPose->m_X[1] = -0.000048908757700053104; 
		m_KinectParams.pPose->m_X[2] = -0.0028166523363169250; */

		////Parameters without distort?
		/*m_KinectParams.depthFu = 585.56866224223631/m_KinectScale;
		m_KinectParams.depthUc = 316.26792237007004/m_KinectScale;
		m_KinectParams.depthFv = 586.60786495871628/m_KinectScale;
		m_KinectParams.depthVc = 246.81163664338146/m_KinectScale;

		m_KinectParams.rgbFu = 521.18355697315383/m_KinectScale;
		m_KinectParams.rgbUc = 319.28799894543192/m_KinectScale;
		m_KinectParams.rgbFv = 522.29636059128597/m_KinectScale;
		m_KinectParams.rgbVc = 259.22861939763811/m_KinectScale;
		
		m_KinectParams.pPose = new CRVL3DPose;
		m_KinectParams.pPose->m_Rot[0] = 0.99999224805435027;
		m_KinectParams.pPose->m_Rot[1] = 0.0024377872532775320;
		m_KinectParams.pPose->m_Rot[2] = 0.0030920906381707190;
		m_KinectParams.pPose->m_Rot[3] = -0.0024318691501572466;
		m_KinectParams.pPose->m_Rot[4] = 0.99999520695564181;
		m_KinectParams.pPose->m_Rot[5] = -0.0019162667297652290;
		m_KinectParams.pPose->m_Rot[6] = -0.0030967472682508152;
		m_KinectParams.pPose->m_Rot[7] = 0.0019087323151372594;
		m_KinectParams.pPose->m_Rot[8] = 0.99999338342676336;
		
		m_KinectParams.pPose->m_X[0] =  0.026667275689053013;
		m_KinectParams.pPose->m_X[1] = 0.000044294836008717494; 
		m_KinectParams.pPose->m_X[2] = 0.0087891702168891236;*/ 	

		////Parameters 3.
		//m_KinectParams.depthFu = 584.70194597700402/m_KinectScale;
		//m_KinectParams.depthUc = 318.55964537649561/m_KinectScale;
		//m_KinectParams.depthFv = 585.70332900816618/m_KinectScale;
		//m_KinectParams.depthVc = 256.14501544470505/m_KinectScale;
		//
		//m_KinectParams.rgbFu = 521.50932707979371/m_KinectScale;
		//m_KinectParams.rgbUc = 322.17660535484873/m_KinectScale;
		//m_KinectParams.rgbFv = 522.56240592356971/m_KinectScale;
		//m_KinectParams.rgbVc = 263.39316821976075/m_KinectScale;
		//
		//m_KinectParams.pPose = new CRVL3DPose;
		//m_KinectParams.pPose->m_Rot[0] = 0.99998448342439450;
		//m_KinectParams.pPose->m_Rot[1] = 0.0027115625740029006;
		//m_KinectParams.pPose->m_Rot[2] = 0.0048662448411574575;
		//m_KinectParams.pPose->m_Rot[3] = -0.0026635059018564940;
		//m_KinectParams.pPose->m_Rot[4] = 0.99994789127814310;
		//m_KinectParams.pPose->m_Rot[5] = -0.0098549715730513066;
		//m_KinectParams.pPose->m_Rot[6] = -0.0048927136394439066;
		//m_KinectParams.pPose->m_Rot[7] = 0.0098418573857854946;
		//m_KinectParams.pPose->m_Rot[8] = 0.99993959777400565;
		//
		//m_KinectParams.pPose->m_X[0] =  0.026546242584187585;
		//m_KinectParams.pPose->m_X[1] = -0.000048908881012111233; 
		//m_KinectParams.pPose->m_X[2] = -0.0028166496710898136;	 
		
		
		//KINECT Params (Cupec)
		m_KinectParams.k = 351218.540681877;
		m_KinectParams.d0 = 1092.93080891449;

		//Create KINECT lookup table for Z coordinate
		m_KinectParams.pZProjLT = new double[2047];
		int i;
		for(i = 0; i < 2047; i++)
			//m_KinectParams.pZProjLT[i] = 0.1236 * tan((i/2842.5) + 1.1863);
			m_KinectParams.pZProjLT[i] = 123.6 * tan((i/2842.5) + 1.1863);

		//Copy KINECT parameters to camera 

		m_pCameraL->fNrm = m_KinectParams.depthFu;
		m_pCameraL->fvNrm = m_KinectParams.depthFv;
		m_pCameraL->CenterXNrm = m_KinectParams.depthUc;
		m_pCameraL->CenterYNrm = m_KinectParams.depthVc;
		m_pCameraL->m_Flags |= RVLCAMERA_FLAG_FV;
	}

	m_ImageWidth = m_pCameraL->Width;
	m_ImageHeight = m_pCameraL->Height;

	m_DisparityMap.Disparity = new short int[m_ImageWidth * m_ImageHeight];
	m_DisparityMap.Width = m_ImageWidth;
	m_DisparityMap.Height = m_ImageHeight;

	m_3DMap = (RVL3DPOINT *)(malloc(m_ImageWidth * m_ImageHeight * 
		sizeof(RVL3DPOINT)));

	m_SADArray = new unsigned short[m_nDisp * m_ImageWidth * m_ImageHeight];

	memset(m_SADArray, 0, m_nDisp * m_ImageWidth * m_ImageHeight * sizeof(short));
}

void CRVLStereoVision::CreateTriangulationLookupTable()
{
	int ImageSize = m_pCameraL->Width * m_pCameraL->Height;

	m_TriangulationLookupTable = new RVLTRIANGULATION_PARAMS[ImageSize];

	RVLTRIANGULATION_PARAMS *pTriangulationParams = m_TriangulationLookupTable;

	double *A1 = m_pCameraL->PRNrm;
	double *c1 = m_pCameraL->InvPRNrm + 9;
	double *A2 = m_pCameraR->PRNrm;
	double *b2 = m_pCameraR->PRNrm + 9;

	m_k1 = (b2[0] - (A2[3 * 0 + 0] * c1[0] + A2[3 * 0 + 1] * c1[1] + A2[3 * 0 + 2] * c1[2])) * 16.0;
	m_k3 = b2[2] - (A2[3 * 2 + 0] * c1[0] + A2[3 * 2 + 1] * c1[1] + A2[3 * 2 + 2] * c1[2]);

	int u, v;
	double fu, fv;
	double P[4], q[2], invP[4];

	for(v = 0; v < m_pCameraL->Height; v++)
	{
		fv = (double)v;

		P[2] = A1[3 * 2 + 0] * fv - A1[3 * 1 + 0];
		P[3] = A1[3 * 2 + 1] * fv - A1[3 * 1 + 1];

		q[1] = A1[3 * 2 + 2] * fv - A1[3 * 1 + 2]; 

		//r[1] = b1[2] * fv - b1[1];

		for(u = 0; u < m_pCameraL->Width; u++)
		{
			fu = (double)(u - m_u1Shift);

			P[0] = A1[3 * 2 + 0] * fu - A1[3 * 0 + 0];
			P[1] = A1[3 * 2 + 1] * fu - A1[3 * 0 + 1];

			q[0] = A1[3 * 2 + 2] * fu - A1[3 * 0 + 2]; 

			//r[0] = b1[2] * fu - b1[0];

			InverseMatrix2(invP, P, APPROX_ZERO);

			pTriangulationParams->f1 = -invP[0] * q[0] - invP[1] * q[1];
			pTriangulationParams->f2 = -invP[2] * q[0] - invP[3] * q[1];

			/*
			pTriangulationParams->g1 = -invP[0] * r[0] - invP[1] * r[1];
			pTriangulationParams->g2 = -invP[2] * r[0] - invP[3] * r[1];

			h1 = b23 + A2[3 * 2 + 0] * pTriangulationParams->g1 + 
				A2[3 * 2 + 1] * pTriangulationParams->g2;
			h2 = b21 + A2[3 * 0 + 0] * pTriangulationParams->g1 + 
				A2[3 * 0 + 1] * pTriangulationParams->g2;
			h3 = A2[3 * 2 + 0] * pTriangulationParams->f1 + 
				A2[3 * 2 + 1] * pTriangulationParams->f2 + A2[3 * 2 + 2];
			h4 = A2[3 * 0 + 0] * pTriangulationParams->f1 + 
				A2[3 * 0 + 1] * pTriangulationParams->f2 + A2[3 * 0 + 2];

			pTriangulationParams->w1 = -h1 / h3;
			pTriangulationParams->w2 = (h1 * fu - h2) / h3 * 16.0;
			pTriangulationParams->w3 = -(fu - h4 / h3) * 16.0;
			*/

			pTriangulationParams->w1 = (A2[3 * 0 + 0] * pTriangulationParams->f1 + 
				A2[3 * 0 + 1] * pTriangulationParams->f2 + A2[3 * 0 + 2]) * 16.0;
			pTriangulationParams->w3 = A2[3 * 2 + 0] * pTriangulationParams->f1 + 
				A2[3 * 2 + 1] * pTriangulationParams->f2 + A2[3 * 2 + 2];

			pTriangulationParams++;
		}
	}
}
/*
void CRVLStereoVision::Get3DWithUncert(unsigned char *EdgeMagnitude, 
									   CRVL3DPose *pPoseLA)
{
	short *pDisparity = m_DisparityMap.Disparity;
	RVL3DPOINT *pP3D = m_3DMap;
	int ImageWidth = m_pCameraL->Width;
	RVLTRIANGULATION_PARAMS *pTriangulationParams = m_TriangulationLookupTable + 
		ImageWidth * m_DisparityMap.v0 + m_DisparityMap.u0;
	unsigned char *pEdgeMagnitude = EdgeMagnitude;

	int u, v, u1, v1, d, u2_16;
	RVLTRIANGULATION_PARAMS *pTriangulationParams2;
	unsigned char *pEdgeMagnitude2;
	double X[3], W3D[3];
	double den;

	for(v = 0; v < m_DisparityMap.Height; v++)
	{
		pTriangulationParams2 = pTriangulationParams;
		pEdgeMagnitude2 = pEdgeMagnitude;

		for(u = 0; u < m_DisparityMap.Width; u++, pDisparity++, pP3D++, pTriangulationParams2++, 
			pEdgeMagnitude2++)
		{
			d = *pDisparity;

			if(d >= m_minDisparity)
			{
				u1 = u + m_DisparityMap.u0;
				v1 = v + m_DisparityMap.v0;

				Triangulation4(u1, v1, d, X, u2_16, den, pTriangulationParams2);

				pPoseLA->Transf(X, pP3D->X);

				GetTriangulationUncert4(den, pTriangulationParams2, W3D);

				pP3D->Wy = pPoseLA->m_Rot[0 + 1 * 3] * W3D[0] + 
					pPoseLA->m_Rot[1 + 1 * 3] * W3D[1] + 
					pPoseLA->m_Rot[2 + 1 * 3] * W3D[2];

				pP3D->u = u1;
				pP3D->v = v1;
				pP3D->d = d + m_DisparityOffsetNrm;

				pP3D->Flags = (RVL3DPOINT_FLAG_3D | RVL3DPOINT_FLAG_W);

				if(*pEdgeMagnitude2 > m_EdgeMagnitudeThr)
					pP3D->Flags |= RVL3DPOINT_FLAG_MASK;
			}
			else
				pP3D->Flags = 0x00;
		}

		pTriangulationParams += ImageWidth;
		pEdgeMagnitude += ImageWidth;
	}
}*/

// Computes 3D coordinates of image points with respect to the 
// coordinate system S_G
/*
void CRVLStereoVision::Create3DMap(CRVL3DPose *pPoseLA)
{
	short *pDisparity = m_DisparityMap.Disparity;
	RVL3DPOINT *pP3D = m_3DMap;
	int ImageWidth = m_pCameraL->Width;
	RVLTRIANGULATION_PARAMS *pTriangulationParams = m_TriangulationLookupTable + 
		ImageWidth * m_DisparityMap.v0 + m_DisparityMap.u0;

	int u, v, u1, v1, d, u2_16;
	RVLTRIANGULATION_PARAMS *pTriangulationParams2;
	double X[3], W3D[3];
	double den;

	for(v = 0; v < m_DisparityMap.Height; v++)
	{
		pTriangulationParams2 = pTriangulationParams;

		for(u = 0; u < m_DisparityMap.Width; u++, pDisparity++, pP3D++, 
			pTriangulationParams2++)
		{
			d = *pDisparity;

			if(d >= m_minDisparity)
			{
				u1 = u + m_DisparityMap.u0;
				v1 = v + m_DisparityMap.v0;

				Triangulation4(u1, v1, d, X, u2_16, den, pTriangulationParams2);

				pPoseLA->Rot(X, pP3D->X);

				GetTriangulationUncert4(den, pTriangulationParams2, W3D);

				pP3D->Wy = pPoseLA->m_Rot[0 + 1 * 3] * W3D[0] + 
					pPoseLA->m_Rot[1 + 1 * 3] * W3D[1] + 
					pPoseLA->m_Rot[2 + 1 * 3] * W3D[2];

				pP3D->u = u1;
				pP3D->v = v1;
				pP3D->d = d + m_DisparityOffsetNrm;

				pP3D->Flags = (RVL3DPOINT_FLAG_3D | RVL3DPOINT_FLAG_W);
			}
			else
				pP3D->Flags = 0x00;
		}

		pTriangulationParams += ImageWidth;
	}
}*/

void CRVLStereoVision::GetMinDisparity()
{
	double X[3];

	X[0] = X[1] = 0.0;
	X[2] = m_maxz;

	double fu1, fv1, fu2, fv2;

	Projection(X, m_pCameraL->PRNrm, fu1, fv1);

	Projection(X, m_pCameraR->PRNrm, fu2, fv2);

	m_minDisparity = (short)(DOUBLE2INT((fu1 - fu2) * 16.0) - m_DisparityOffsetNrm);	

	// only for debugging purpose

	/*
	int u1 = DOUBLE2INT(fu1);
	int v1 = DOUBLE2INT(fv1);

	RVLTRIANGULATION_PARAMS *pTriangulationParams = m_TriangulationLookupTable + 
		u1 + v1 * m_BmpWidth;	

	double P3D2[3];
	int u2_16;
	double den;

	Triangulation4(u1, v1, m_minDisparity, P3D2, u2_16, den, pTriangulationParams);

	double W3D[3];

	GetTriangulationUncert4(den, pTriangulationParams, W3D);
	*/
}

void CRVLStereoVision::DisplayHeightMap(PIX_ARRAY &HeightPixArray, 
										double y0)
{
	int n = HeightPixArray.Width * HeightPixArray.Height;
	int maxh = 15;
	int k1 = 200 / (2 * maxh);

	int iPix;
	unsigned char *pPix = HeightPixArray.pPix;
	RVL3DPOINT *pP3D = m_3DMap;
	int h;
	double yCorr;

	for(iPix = 0; iPix < n; iPix++)
	{
	    if(pP3D->Flags)
		{
			if(m_3DMapRotCorr != NULL)
				yCorr = m_3DMapRotCorr[3] * pP3D->X[0] + 
						m_3DMapRotCorr[4] * pP3D->X[1] + 
						m_3DMapRotCorr[5] * pP3D->X[2];
			else
				yCorr = pP3D->X[1];

			h = DOUBLE2INT(yCorr - y0);

			/*
			if(h >= -maxh && h <= maxh)
				*pPix = (unsigned char)(255 - (h + maxh) * k1);
			//else if(h > 50)
			//	*pPix = (unsigned char)(255 - (50 + 50) * k1);
			else
				*pPix = 0;
			*/

			if(h < -maxh)
				//*pPix = 55;
				*pPix = 0;
			else if(h <= maxh)
				*pPix = 55 + (maxh - h) * 100 / maxh;
				//*pPix = 192;
			else
				//*pPix = 255;
				*pPix = 0;
		}
	    else
			*pPix = 0;

	    pPix++;
	    pP3D++;
	}
}


void CRVLStereoVision::CreateDisparityMap()
{

#ifdef SVS
	if(m_Flags & RVLSTEREO_FLAGS_METHOD_SVS)
	{
		svsStereoImage *pStereoImage = (svsStereoImage *)m_vpStereoImage;

		pStereoImage->haveImages = TRUE;
		pStereoImage->haveColor = FALSE;
		pStereoImage->haveColorRight = FALSE;
		pStereoImage->isRectified = TRUE;
		pStereoImage->haveRect = FALSE;
		pStereoImage->haveDisparity = FALSE;  
		pStereoImage->ip.vergence = 0;
		pStereoImage->dp.lr = TRUE;	

		pStereoImage->SetImages(m_pCameraL->m_nrmImage.pPix, 
			m_pCameraR->m_nrmImage.pPix, NULL, NULL);
		
		GetDisparityOffset();
		
		pStereoImage->ip.linelen = m_pCameraL->m_nrmImage.Width;
		pStereoImage->ip.lines = m_pCameraL->m_nrmImage.Height;
		pStereoImage->ip.ix = 0;
		pStereoImage->ip.iy = 0;
		pStereoImage->ip.width = m_pCameraL->m_nrmImage.Width;
		pStereoImage->ip.height = m_pCameraL->m_nrmImage.Height;
		pStereoImage->ip.vergence = 0;
		pStereoImage->rp.left.pwidth = m_pCameraL->m_nrmImage.Width;
		pStereoImage->rp.left.pheight = m_pCameraL->m_nrmImage.Height;
		pStereoImage->rp.right.pwidth = m_pCameraR->m_nrmImage.Width;
		pStereoImage->rp.right.pheight = m_pCameraR->m_nrmImage.Height;
		pStereoImage->dp.corrsize = m_WinSize;
		pStereoImage->dp.thresh = m_ConfThr;
		//pStereoImage->dp.unique = m_UniquenessThr;
		pStereoImage->dp.ndisp = m_nDisp;
		pStereoImage->dp.dpp = m_nSubPixDisps;
		pStereoImage->dp.offx = -m_DisparityOffset;
		pStereoImage->dp.dleft = 0;
		pStereoImage->dp.dtop = 0;
		pStereoImage->dp.dwidth = m_pCameraL->m_nrmImage.Width;
		pStereoImage->dp.dheight = m_pCameraL->m_nrmImage.Height;

		svsStereoProcess StereoProcess;

		StereoProcess.CalcStereo(pStereoImage);

		m_DisparityMap.Disparity = pStereoImage->Disparity();
		m_DisparityMap.Width = m_pCameraL->m_nrmImage.Width;
		m_DisparityMap.Height = m_pCameraL->m_nrmImage.Height;
		m_DisparityMap.u0 = 0;
		m_DisparityMap.v0 = 0;	

		if(m_DisparityFileName)
		{
			pStereoImage->SaveDisparity(m_DisparityFileName);
			
			pStereoImage->SaveParams(m_ParamFileName);
		}
	}
	else
#endif

	if(m_Flags & RVLSTEREO_FLAGS_METHOD_OPENCV)
	{
		IplImage *pDisparityImage = cvCreateImageHeader(cvSize(m_pCameraL->m_nrmImage.Width, m_pCameraL->m_nrmImage.Height) ,IPL_DEPTH_16S,1);

		pDisparityImage->imageData = (char *)(m_DisparityMap.Disparity);

		m_DisparityMap.Width = m_pCameraL->m_nrmImage.Width;
		m_DisparityMap.Height = m_pCameraL->m_nrmImage.Height;
		m_DisparityMap.u0 = 0;
		m_DisparityMap.v0 = 0;	


		//CvStereoBMState* state = cvCreateStereoBMState(CV_STEREO_BM_BASIC);

		//Set params
		/*state->SADWindowSize = m_WinSize;		//5
		state->preFilterSize = 5;
		state->numberOfDisparities = m_nDisp;	//64
		state->textureThreshold = m_ConfThr;	//10
		state->uniquenessRatio = 15;*/

		IplImage* pLeftImage = cvCreateImageHeader( cvSize(m_pCameraL->m_nrmImage.Width, m_pCameraL->m_nrmImage.Height), IPL_DEPTH_8U, 1  );
		IplImage* pRightImage = cvCreateImageHeader( cvSize(m_pCameraR->m_nrmImage.Width, m_pCameraR->m_nrmImage.Height), IPL_DEPTH_8U, 1  );

		pLeftImage->imageData = (char *)m_pCameraL->m_nrmImage.pPix;

		pRightImage->imageData = (char *)m_pCameraR->m_nrmImage.pPix;

		//pDisparityImage = cvCreateImage(cvSize(m_pCameraL->m_nrmImage.Width, m_pCameraL->m_nrmImage.Height),IPL_DEPTH_16S,1);

		//pDisparityImage->imageData = (char *)malloc(450*375*2);

#ifdef RVLSTEREO_CORRESPONDENCE_SAD_ARRAY
		cvFindStereoCorrespondenceBM_RVL(pLeftImage, pRightImage, pDisparityImage,state, m_SADArray);
#else
		//cvFindStereoCorrespondenceBM(pLeftImage, pRightImage, pDisparityImage,state);
#endif

		//cvReleaseStereoBMState(&state);
		cvReleaseImageHeader(&pDisparityImage);
		cvReleaseImageHeader(&pLeftImage);
		cvReleaseImageHeader(&pRightImage);
	
	}
}


#ifdef SVS
void CRVLStereoVision::NormalizeStereoImage(PIX_ARRAY *pImageL, 
											PIX_ARRAY *pImageR, 
											PIX_ARRAY *pNrmImageL, 
											PIX_ARRAY *pNrmImageR)
{
	svsStoredImages *pImageAcquisition = 
		(svsStoredImages *)m_vpImageAcquisition;

	pImageAcquisition->Load(pImageL->Width, pImageL->Height,
		pImageL->pPix, pImageR->pPix, NULL, NULL, FALSE, TRUE);

	svsStereoImage *pStereoImage = pImageAcquisition->GetImage(0);

	pNrmImageL->pPix = pStereoImage->Left();
	pNrmImageL->bOwnData = FALSE;
	pNrmImageR->pPix = pStereoImage->Right();
	pNrmImageR->bOwnData = FALSE;	
}

#endif

void CRVLStereoVision::GetDisparityOffset()
{
	DWORD Method = (m_Flags & RVLSTEREO_FLAGS_DISPARITY_OFFSET);

	if(Method == RVLSTEREO_FLAGS_DISPARITY_OFFSET_AUTO)
	{
		double BaseLen = m_pCameraR->InvPRNrm[9] - m_pCameraL->InvPRNrm[9];

		m_DisparityOffset = DOUBLE2INT(m_pCameraL->fNrm * BaseLen / m_minDist -
			m_pCameraR->CenterXNrm +
			m_pCameraL->CenterXNrm - m_nDisp); 

		
	}
	else if(Method == RVLSTEREO_FLAGS_DISPARITY_OFFSET_FILE)
	{
		// add code here when you find time!
	}
}

void CRVLStereoVision::SetDisparityOffset(int DisparityOffset)
{
	m_DisparityOffset = DisparityOffset;

	m_DisparityOffsetNrm = (m_DisparityOffset << 4);
}


void CRVLStereoVision::InitCamerasSVS(char *ParamFileName)
{
	CRVLCamera *pCamera[2];

	pCamera[0] = m_pCameraL;
	pCamera[1] = m_pCameraR;

	CRVLCamera *pCamera2;

	int iCamera, i, j;
	double *pPR;

#ifdef SVS
	svsStoredImages *pImageAcquisition = (svsStoredImages *)m_vpImageAcquisition;

	pImageAcquisition->ReadParams(ParamFileName);

	svsIntrinsicParams *pCameraParams[2];

	pCameraParams[0] = &(pImageAcquisition->GetRP()->left);
	pCameraParams[1] = &(pImageAcquisition->GetRP()->right);

	svsIntrinsicParams *pCameraParams2;

	for(iCamera = 0; iCamera < 2; iCamera++)
	{
		pCamera2 = pCamera[iCamera];
		pCameraParams2 = pCameraParams[iCamera];

		pPR = pCamera2->PRNrm;

		for(i = 0; i < 3; i++)
			for(j = 0; j < 3; j++, pPR++)
				*pPR = (double)(pCameraParams2->proj[i][j]);

		for(i = 0; i < 3; i++, pPR++)
			*pPR = (double)(pCameraParams2->proj[i][3]);

		pCamera2->Width = pCamera2->m_Image.Width = 
			pCamera2->m_nrmImage.Width = pCameraParams2->pwidth;
		pCamera2->Height = pCamera2->m_Image.Height = 
			pCamera2->m_nrmImage.Height = pCameraParams2->pheight;

		pCamera2->m_Image.pPix = pCamera2->m_nrmImage.pPix = NULL;
		pCamera2->m_Image.bOwnData = pCamera2->m_nrmImage.bOwnData = FALSE;
		pCamera2->m_Image.bColor = pCamera2->m_nrmImage.bColor = FALSE;		

		pCamera2->fNrm = pCamera2->PRNrm[0];
		pCamera2->CenterXNrm = pCamera2->PRNrm[2];
		pCamera2->CenterYNrm = pCamera2->PRNrm[5];

		InverseMatrix3(pCamera2->InvPRNrm, pCamera2->PRNrm);

		LinearTransform3D(pCamera2->InvPRNrm, pCamera2->PRNrm + 9, 
			pCamera2->InvPRNrm + 9);

		pCamera2->InvPRNrm[9]  = -pCamera2->InvPRNrm[9];
		pCamera2->InvPRNrm[10] = -pCamera2->InvPRNrm[10];
		pCamera2->InvPRNrm[11] = -pCamera2->InvPRNrm[11];
	}
#else
	FILE *fp = fopen(ParamFileName, "r");

	char line[200];

	iCamera = -1;
	BOOL bReadWidthAndHeight = FALSE;

	while(!feof(fp))
	{
		fgets(line, 200, fp);

		if(strcmp(line, "[left camera]\xa") == 0)
		{
			iCamera = 0;
			bReadWidthAndHeight = TRUE;
		}
		else if(strcmp(line, "[right camera]\xa") == 0)
		{
			iCamera = 1;
			bReadWidthAndHeight = TRUE;
		}
		else if(strcmp(line, "proj \xa") == 0)
		{			
			pPR = pCamera2->PRNrm;

			for(i = 0; i < 3; i++)
			{
				fscanf(fp, " ");

				for(j = 0; j < 3; j++, pPR++)
					fscanf(fp, " %lf", pPR);

				fscanf(fp, " %lf", pCamera2->PRNrm + 9 + i);

				fscanf(fp, "\n");
			}

			pCamera2->fNrm = pCamera2->fvNrm = pCamera2->PRNrm[0];
			pCamera2->CenterXNrm = pCamera2->PRNrm[2];
			pCamera2->CenterYNrm = pCamera2->PRNrm[5];

			InverseMatrix3(pCamera2->InvPRNrm, pCamera2->PRNrm);

			LinearTransform3D(pCamera2->InvPRNrm, pCamera2->PRNrm + 9, pCamera2->InvPRNrm + 9);

			pCamera2->InvPRNrm[9]  = -pCamera2->InvPRNrm[9];
			pCamera2->InvPRNrm[10] = -pCamera2->InvPRNrm[10];
			pCamera2->InvPRNrm[11] = -pCamera2->InvPRNrm[11];
		
			if(iCamera == 1)
				break;
		}

		if(bReadWidthAndHeight)
		{
			bReadWidthAndHeight = FALSE;

			pCamera2 = pCamera[iCamera];

			fscanf(fp, "pwidth %d \n", &(pCamera2->Width));
			fscanf(fp, "pheight %d \n", &(pCamera2->Height));

			pCamera2->m_nrmImage.Width = pCamera2->m_Image.Width = pCamera2->Width;
			pCamera2->m_nrmImage.Height = pCamera2->m_Image.Height = pCamera2->Height;
			pCamera2->m_nrmImage.bColor = pCamera2->m_Image.bColor = FALSE;
		}
	}

	fclose(fp);
#endif
}

void CRVLStereoVision::CreateStereoPointArray()
{
	if(m_Flags & RVLSTEREO_STEREO_POINT_ARRAY)
		return;

	short int *pDisparity = m_DisparityMap.Disparity;
	RVLSTEREOPOINT *pStereoPoint = m_StereoPointArray;

	int u, v;

	for(v = 0; v < m_DisparityMap.Height; v++)
		for(u = 0; u < m_DisparityMap.Width; u++)
			if(*pDisparity >= 0)
			{
				pStereoPoint->disparity = *pDisparity;
				pStereoPoint->u = u;
				pStereoPoint->v = v;
				pStereoPoint++;
			}

	m_nStereoPoints = pStereoPoint - m_StereoPointArray;

	m_Flags |= RVLSTEREO_STEREO_POINT_ARRAY;
}

void CRVLStereoVision::Statistics(float *dGuI, 
								  float *dGvI)
{
	int w = m_pCameraL->m_nrmImage.Width;
	int h = m_pCameraL->m_nrmImage.Height;
	int imageSize = w * h;

	//CRVLHistogram histeI, histeIu, histeIv;
	
	/*histeI.Init(2.0, m_pMem2, "Intensity");
	histeIu.Init(2.0, m_pMem2, "Intensity Gradient u");
	histeIv.Init(2.0, m_pMem2, "Intensity Gradient v");*/

	short int *D = m_DisparityMap.Disparity;
	unsigned char *I1 = m_pCameraL->m_nrmImage.pPix;
	unsigned char *I2 = m_pCameraR->m_nrmImage.pPix;

	int u1, v1;
	int iPix1, iPix2;
	short int d;
	double eI;
	double eIu;
	//double eIv;

	for(v1 = 3; v1 <= 235; v1++)
		for(u1 = 69; u1 <= 244; u1++)
		{
			iPix1 = u1 + v1 * w;

			d = D[iPix1];

			if(d < 0)
				continue;

			d = ((d + 8) >> 4);

			iPix2 = iPix1 - d;

			eI = (double)((int)I1[iPix1] - (int)I2[iPix2]);

			//histeI.m_Data.Add(&eI);

			eIu = (double)(dGuI[iPix1]);

			//histeIu.m_Data.Add(&eIu);

			//if(fabs(eI) > 10.0)
			//	D[iPix1] = -1;

			//if(dGuI[iPix1] * dGuI[iPix1] + dGvI[iPix1] * dGvI[iPix1] < 10.0)
			//	D[iPix1] = -1;
		}

	/*histeI.Create();
	histeIu.Create();

	FILE *fp = fopen("C:\\RVL\\ExpRez\\StereoVisionStatistics.txt", "w");*/

	/*histeI.Save(fp);
	histeIu.Save(fp);

	fclose(fp);*/

	m_pMem2->Clear();
}


// Mathematics behind this function is given in from_disparity_space_to_3D.doc

void CRVLStereoVision::GetUVDPlane(	double *N,
									double d,
									double &a,
									double &b,
									double &c)
{
	if((m_Flags & RVLSTEREO_FLAGS_METHOD) == RVLSTEREO_FLAGS_METHOD_KINECT)
	{
		double k1 = -m_KinectParams.k / d;

		a = k1 * N[0] / m_KinectParams.depthFu;
		b = k1 * N[1] / m_KinectParams.depthFv;
		c = k1 * N[2] - a * m_KinectParams.depthUc - b * m_KinectParams.depthVc + m_KinectParams.d0;
	}
	else
	{
		double k1 = m_BaseLenNrm / d;

		a = k1 * N[0];
		b = k1 * N[1];
		c = k1 * (-N[0] * m_pCameraL->CenterXNrm - N[1] * m_pCameraL->CenterYNrm + N[2] * m_pCameraL->fNrm);
	}
}

void CRVLStereoVision::SADStatistics()
{
	short *pDisparityMapEnd = m_DisparityMap.Disparity + m_ImageWidth * m_ImageHeight;

	short *pDisp;
	short d0;

	// find SAD range

	unsigned short maxSAD = 0;

	unsigned short *SAD = m_SADArray;

	int iminSAD;
	unsigned short minSAD;

	for(pDisp = m_DisparityMap.Disparity; pDisp < pDisparityMapEnd; pDisp++, SAD += m_nDisp)
	{
		d0 = *pDisp;

		if(d0 < 0)
			continue;

		iminSAD = m_nDisp - 1 - ((d0 >> 4) + ((d0 & 15) > 8));

		minSAD = SAD[iminSAD];

		//if(iminSAD > 0)
		//	if(SAD[iminSAD - 1] < minSAD)
		//		minSAD = SAD[iminSAD - 1];

		//if(iminSAD < m_nDisp - 1)
		//	if(SAD[iminSAD + 1] < minSAD)
		//		minSAD = SAD[iminSAD + 1];

		//for(d = 0; d < m_nDisp; d++)
		//	if(SAD[d] < minSAD)
		//		int tmp1 = 0;

		if(minSAD > maxSAD)
			maxSAD = minSAD;
	}

	int *Hist = new int[maxSAD + 1];

	memset(Hist, 0, (maxSAD + 1) * sizeof(int));

	int n = 0;

	SAD = m_SADArray;

	for(pDisp = m_DisparityMap.Disparity; pDisp < pDisparityMapEnd; pDisp++, SAD += m_nDisp)
	{
		d0 = *pDisp;

		if(d0 < 0)
			continue;

		iminSAD = m_nDisp - 1 - (d0 >> 4);

		minSAD = SAD[iminSAD];

		if(iminSAD > 0)
			if(SAD[iminSAD - 1] < minSAD)
				minSAD = SAD[iminSAD - 1];

		if(iminSAD < m_nDisp - 1)
			if(SAD[iminSAD + 1] < minSAD)
				minSAD = SAD[iminSAD + 1];

		Hist[minSAD]++;

		n++;
	}

	FILE *fp;

	fp = fopen("C:\\RVL\\ExpRez\\SASStatistics.dat", "w");

	int SAD2;

	int cumsum = 0;

	for(SAD2 = 0; SAD2 <= maxSAD; SAD2++)
	{
		cumsum += Hist[SAD2];

		fprintf(fp, "%d\t%lf\n", SAD2, (double)(100 * cumsum)/(double)n);
	}

	fclose(fp);

	delete[] Hist;
}


void CRVLStereoVision::CreateParamList(CRVLMem *pMem)
{
	m_ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	m_ParamList.Init();

	pParamData = m_ParamList.AddParam("StereoVision.CameraFileName", RVLPARAM_TYPE_STRING, 
		&m_ParamFileName);

	pParamData = m_ParamList.AddParam("StereoVision.Method", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "SVS", RVLSTEREO_FLAGS_METHOD_SVS);
	m_ParamList.AddID(pParamData, "OPENCV", RVLSTEREO_FLAGS_METHOD_OPENCV);
	m_ParamList.AddID(pParamData, "KINECT", RVLSTEREO_FLAGS_METHOD_KINECT);
	m_ParamList.AddID(pParamData, "SVS_WARP", RVLSTEREO_FLAGS_CALIB_METHOD_SVS);

	pParamData = m_ParamList.AddParam("StereoVision.DisparityRange", RVLPARAM_TYPE_INT, &m_nDisp);

	pParamData = m_ParamList.AddParam("StereoVision.WindowSize", RVLPARAM_TYPE_INT, &m_WinSize);

	pParamData = m_ParamList.AddParam("StereoVision.ConfidenceThr", RVLPARAM_TYPE_INT, &m_ConfThr);

	pParamData = m_ParamList.AddParam("StereoVision.Kinect.Scale", RVLPARAM_TYPE_INT, &m_KinectScale);

	pParamData = m_ParamList.AddParam("StereoVision.Width", RVLPARAM_TYPE_INT, &(m_pCameraL->Width));
	pParamData = m_ParamList.AddParam("StereoVision.Height", RVLPARAM_TYPE_INT, &(m_pCameraL->Height));
	pParamData = m_ParamList.AddParam("StereoVision.Kinect.fu", RVLPARAM_TYPE_DOUBLE, &(m_KinectParams.depthFu0));
	pParamData = m_ParamList.AddParam("StereoVision.Kinect.fv", RVLPARAM_TYPE_DOUBLE, &(m_KinectParams.depthFv0));
	pParamData = m_ParamList.AddParam("StereoVision.Kinect.uc", RVLPARAM_TYPE_DOUBLE, &(m_KinectParams.depthUc0));
	pParamData = m_ParamList.AddParam("StereoVision.Kinect.vc", RVLPARAM_TYPE_DOUBLE, &(m_KinectParams.depthVc0));

}

void CRVLStereoVision::Get3DKinect(int u, int v, int d, double *X)
{
	X[2] = m_KinectParams.pZProjLT[d];
	X[0] = (u - m_KinectParams.depthUc) * (X[2]/m_KinectParams.depthFu);
	X[1] = (v - m_KinectParams.depthVc) * (X[2]/m_KinectParams.depthFv);
}

void CRVLStereoVision::GetKinectProjectionMatrix(double *P)
{
	P[0] = m_KinectParams.depthFu;
	P[1] = 0.0;
	P[2] = m_KinectParams.depthUc;
	P[3] = 0.0;
	P[4] = m_KinectParams.depthFv;
	P[5] = m_KinectParams.depthVc;
	P[6] = 0.0;
	P[7] = 0.0;
	P[8] = 1.0;
}



void RVLDisplayDisparityMap(RVLDISPARITYMAP *pDisparityMap, 
							BOOL bInverse, 
							PIX_ARRAY &DisparityPixArray)
{
	DisparityPixArray.Width = pDisparityMap->Width;
	DisparityPixArray.Height = pDisparityMap->Height;

	int n = pDisparityMap->Width * pDisparityMap->Height;

	int iPix;
	unsigned char *pPix = DisparityPixArray.pPix;
	short *pDisparity = pDisparityMap->Disparity;

	if(bInverse)
	{
		for(iPix = 0; iPix < n; iPix++)
		{
		    if(*pDisparity >= 0)
				*pPix = 255 - (unsigned char)((*pDisparity) / 4);
		    else
				*pPix = 255;

		    pPix++;
		    pDisparity++;
		}
	}
	else
	{
		for(iPix = 0; iPix < n; iPix++)
		{
		    if(*pDisparity >= 0)
				*pPix = (unsigned char)((*pDisparity) / 4);
		    else
				*pPix = 0;

		    pPix++;
		    pDisparity++;
		}
	}
}




#ifdef SVS
WORD CRVLStereoVision::CameraInit()
{
	m_vpVideoObject = (void *)getVideoObject();

	svsVideoImages *pVideoObject = (svsVideoImages *)m_vpVideoObject;

	bool ret = pVideoObject->Open();

	if (!ret)
		return RVLSTEREO_RES_CANT_OPEN_CAMERA;

	if(pVideoObject->ReadParams("C:\\RVL\\Calibration\\Calib_081124\\megad-75.ini"))  
		pVideoObject->SaveParams("C:\\RVL\\Calibration\\Calib_081124\\megad-75.dat"); 
	else
		return RVLSTEREO_RES_CANT_LOAD_PARAMS;

	if(!pVideoObject->SetRate(3)) //3   15
		return RVLSTEREO_RES_CANT_SET_FRAME_RATE;

	int width=320, height=240;
	//svsWindow *win1 = new svsWindow(width,height);
	//svsWindow *win2 = new svsWindow(width,height);
	pVideoObject->SetSize(width,height);

	pVideoObject->SetExposure(20, 15, TRUE, TRUE);

	if(!pVideoObject->Start())
		return RVLSTEREO_RES_CANT_START_ACQUISITION;

	return RVL_RES_OK;
}

WORD CRVLStereoVision::ImageAcquisition(int n)
{
	svsVideoImages *pVideoObject = (svsVideoImages *)m_vpVideoObject;

	svsStereoImage *imageObject;

	int i;

	for(i = 0; i < n; i++)
		imageObject = pVideoObject->GetImage(100);  //100 200
	
	m_vpStereoImage = (void *)imageObject;
		
	if(!imageObject)   //imageObject == NULL
		return RVLSTEREO_RES_IMAGE_ACQUISITION_FAILED;

	if(m_pCameraL->m_Image.bOwnData)
		if(m_pCameraL->m_Image.pPix)
			delete[] m_pCameraL->m_Image.pPix;

	m_pCameraL->m_Image.bOwnData = FALSE;
	m_pCameraL->m_Image.pPix = imageObject->Left();
	m_pCameraL->m_Image.Width = 320;
	m_pCameraL->m_Image.Height = 240;

	if(m_pCameraR->m_Image.bOwnData)
		if(m_pCameraR->m_Image.pPix)
			delete[] m_pCameraR->m_Image.pPix;

	m_pCameraR->m_Image.bOwnData = FALSE;
	m_pCameraR->m_Image.pPix = imageObject->Right();
	m_pCameraR->m_Image.Width = 320;
	m_pCameraR->m_Image.Height = 240;		

	return RVL_RES_OK;
}

void CRVLStereoVision::CameraStop()
{
	if(m_vpVideoObject)
	{
		svsVideoImages *pVideoObject = (svsVideoImages *)m_vpVideoObject;

		pVideoObject->Stop();
	}
}
#endif	// SVS

/*void RVLSaveStereoPts(FILE *fp,
					  RVL3DPOINT2 *PtArray,
					  int nPts)
{
	int i;
	RVL3DPOINT2 *pPt;

	for(i = 0; i < nPts; i++)
	{
		pPt = PtArray + i;

		fprintf(fp, "%lf\t%lf\t%lf\n", pPt->x, pPt->y, pPt->z);
	}
}*/


//Get xyz coordinates from uvd values
void RVLGetKinect3DData(int *pUVDPt, double *pXYZPt, RVLKINECT_PARAMS kinectParams)
{
	//new method (Cupec)
	// pXYZPt[2] = kinectParams.k / (kinectParams.d0 - pUVDPt[2]);
	//z
	pXYZPt[2] = kinectParams.pZProjLT[pUVDPt[2]];
	//x
	pXYZPt[0] = (pUVDPt[0] - kinectParams.depthUc) * (pXYZPt[2]/kinectParams.depthFu);
	//y
	pXYZPt[1] = (pUVDPt[1] - kinectParams.depthVc) * (pXYZPt[2]/kinectParams.depthFv);

}



// RVLGUI.cpp: implementation of the CRVLGUI class.
//
//////////////////////////////////////////////////////////////////////

#include "highgui.h"
#include "RVLCore.h"
//#include "RVLObjectLib.h"
#include "RVLGUI.h"
#ifdef RVLOPENNI
#include "OpenNI.h"
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLGUI::CRVLGUI()
{
	m_Flags = 0x00000000;
	m_nSequenceSamples = 1;
	m_pSelectedObject = NULL;
	m_pSelectedObject2 = NULL;
	m_SampleDirectory = NULL;
	m_OutputDirectory = NULL;
	m_InputImageFileName = NULL;
	m_iStep = 0;
	m_nSteps = 0;
	m_nOutImgVers = 1;
	m_OutImgVer = 0; 
	m_nTextLines = 0;
	//m_pMutex = NULL;

	// generating RVLGUIOpticalFlowDisplayColorLT

	RVLCOLOR *pColor = m_OpticalFlowDisplayColorLT;

	double k = 0.5 / RVLGUI_OPTICAL_FLOW_DISPLAY_COLOR_LT_HALF_SIZE;

	int du, dv;
	double fdu, fdv, hrad, l;

	for(dv = -RVLGUI_OPTICAL_FLOW_DISPLAY_COLOR_LT_HALF_SIZE; 
		dv <= RVLGUI_OPTICAL_FLOW_DISPLAY_COLOR_LT_HALF_SIZE; dv++)
		for(du = -RVLGUI_OPTICAL_FLOW_DISPLAY_COLOR_LT_HALF_SIZE; 
			du <= RVLGUI_OPTICAL_FLOW_DISPLAY_COLOR_LT_HALF_SIZE; du++, pColor++)
		{
			fdu = (double)du;
			fdv = (double)dv;

			hrad = atan2(fdv, fdu);

			l = k * sqrt(fdu * fdu + fdv * fdv);

			if(l > 0.5)
				l = 0.5;

			*pColor = RVLHSL2RGB(hrad, 1.0, l);
		}

	//m_PoseC0ChangeScale.m_X[0] = 100.0;
}

CRVLGUI::~CRVLGUI()
{
	Clear();

	if(m_SampleDirectory)
		delete[] m_SampleDirectory;

	if(m_OutputDirectory)
		delete[] m_OutputDirectory;

	if(m_InputImageFileName)
		delete[] m_InputImageFileName;

	//if(m_pMutex)
	//	delete m_pMutex;
}

DWORD CRVLGUI::Init(char *CfgFileName)
{
	/*FILE *fp = fopen(CfgFileName, "r");

	if(!fp)
		return RVLGUI_INIT_RES_ERR_CFG_FILE;

	fscanf(fp, "Init config=%s\n", m_ParamFileName);

	fclose(fp);*/

	//m_ParamList.LoadParams(m_ParamFileName);

	//m_Figure[0].Create(m_pMem);
	//m_Figure[1].Create(m_pMem);
	m_Figure[0].Create(1000000);
	m_Figure[1].Create(1000000);

	m_Figure[0].m_vpGUI = m_Figure[1].m_vpGUI = this;

	//m_FigureList.m_pMem = m_pMem0;

	return RVL_RES_OK;
}

DWORD CRVLGUI::Init()
{
	//m_FigureList.m_pMem = m_pMem0;

	return RVL_RES_OK;
}

void CRVLGUI::Clear()
{	
	m_ParamList.Clear();

	ClearDisplay();

	cvDestroyAllWindows();

	CRVLFigure *pFig;

	//m_FigureList.Start();

	/*while(m_FigureList.m_pNext)
	{
		pFig = (CRVLFigure *)(m_FigureList.GetNext());

		delete pFig;
	}

	m_FigureList.RemoveAll();*/
}


void CRVLGUI::CreateParamList()
{
	m_ParamList.m_pMem = m_pMem0;

	m_ParamList.Init();

	m_ParamList.AddParam("Sample directory", RVLPARAM_TYPE_STRING, 
		&m_SampleDirectory);
	m_ParamList.AddParam("Input image", RVLPARAM_TYPE_STRING, 
		&m_InputImageFileName);
	m_ParamList.AddParam("Output directory", RVLPARAM_TYPE_STRING, 
		&m_OutputDirectory);
	m_ParamList.AddParam("Sequence - Number of Samples", RVLPARAM_TYPE_INT, &m_nSequenceSamples);
}

void CRVLGUI::ClearDisplay()
{
	if((m_Figure[0].m_Flags & RVLFIG_FLAG_VECTORS_LOCKED) == 0)
		m_Figure[0].Clear();

	if((m_Figure[1].m_Flags & RVLFIG_FLAG_VECTORS_LOCKED) == 0)
		m_Figure[1].Clear();

	m_pMem->Clear();
}


CRVLFigure * CRVLGUI::GetFigure(DWORD ID)
{
	CRVLFigure *pFig;

	//m_FigureList.Start();

	/*while(m_FigureList.m_pNext)
	{
		pFig = (CRVLFigure *)(m_FigureList.GetNext());

		if(pFig->m_ID == ID)
			return pFig;
	}*/

	return NULL;
}

void CRVLGUI::DisplayVectors(	CRVLFigure *pFig,
								int u0, int v0,
								double ZoomFactor)
{
	CRVLDisplayVector *pVector;

	pFig->m_VectorArray.Start();

	while(pFig->m_VectorArray.m_pNext)
	{
		pVector = (CRVLDisplayVector *)(pFig->m_VectorArray.GetNext());

		DisplayVector(pVector, pFig->m_pImage, u0, v0, ZoomFactor);
	}	
}

CRVLFigure * CRVLGUI::OpenFigure(	char *ImageName,
									int MemSize)
{
	CRVLFigure *pFig;

	BOOL bExists = FALSE;

	//m_FigureList.Start();

	/*while(m_FigureList.m_pNext)
	{
		pFig = (CRVLFigure *)(m_FigureList.GetNext());

		if(bExists = (strcmp(ImageName, pFig->m_ImageName) == 0))
			break;
	}

	if(!bExists)
	{
		pFig = new CRVLFigure;

		pFig->Create(MemSize);

		m_FigureList.Add(pFig);

		pFig->m_vpGUI = this;

		pFig->m_ImageName = RVLCreateString(ImageName);
	}*/

	return pFig;
}

void CRVLGUI::DisplayVector(	CRVLDisplayVector *pVector,
								IplImage *pImage,
								int u0, int v0,
								double ZoomFactor)
{
	if(pVector->m_PointArray.m_nElements == 0)
		return;

	pVector->m_PointArray.Start();

	RVLGUI_POINT *pP0 = (RVLGUI_POINT *)(pVector->m_PointArray.GetNext());	

	RVLGUI_POINT *pP1 = pP0;

	//MarkPoint(pClientDC, CPoint((pP0->u >> 1) + u0, (pP0->v >> 1) + v0), pVector->m_PointType, ColorP);

	if(pVector->m_PointArray.m_pNext == NULL)
		return;

	//COLORREF ColorL;

	//ColorL = RGB(pVector->m_rL, pVector->m_gL, pVector->m_bL);

	RVLGUI_POINT *pP2;

	while(pVector->m_PointArray.m_pNext)
	{
		pP2 = (RVLGUI_POINT *)(pVector->m_PointArray.GetNext());

		cvLine(pImage, 
			cvPoint((int)(ZoomFactor * ((double)((pP1->u >> 1) + u0) + 0.5)),
			(int)(ZoomFactor * ((double)((pP1->v >> 1) + v0) + 0.5))),
			cvPoint((int)(ZoomFactor * ((double)((pP2->u >> 1) + u0) + 0.5)),
			(int)(ZoomFactor * ((double)((pP2->v >> 1) + v0) + 0.5))), 
			cvScalar(pVector->m_bL, pVector->m_gL, pVector->m_rL), pVector->m_LineWidth);

		//MarkPoint(pClientDC, CPoint((pP->u >> 1) + u0, (pP->v >> 1) + v0), pVector->m_PointType, ColorP);

		pP1 = pP2;
	}

	if(pVector->m_bClosed)
		cvLine(pImage, 
			cvPoint((int)(ZoomFactor * ((double)((pP2->u >> 1) + u0) + 0.5)),
			(int)(ZoomFactor * ((double)((pP2->v >> 1) + v0) + 0.5))),
			cvPoint((int)(ZoomFactor * ((double)((pP0->u >> 1) + u0) + 0.5)),
			(int)(ZoomFactor * ((double)((pP0->v >> 1) + v0) + 0.5))), 
			cvScalar(pVector->m_bL, pVector->m_gL, pVector->m_rL), pVector->m_LineWidth);
}

void CRVLGUI::ShowFigure(CRVLFigure *pFig)
{
	cvShowImage(pFig->m_ImageName, pFig->m_pImage);
}


void CRVLGUI::CloseFigure(char * ImageName)
{
	CRVLFigure *pFig;

	BOOL bExists = FALSE;

	//RVLPTRCHAIN_ELEMENT *pCurrent;

	/*m_FigureList.Start();

	while(m_FigureList.m_pNext)
	{
		pCurrent = m_FigureList.m_pCurrent;

		pFig = (CRVLFigure *)(m_FigureList.GetNext());

		if(bExists = (strcmp(ImageName, pFig->m_ImageName) == 0))
		{
			cvDestroyWindow(ImageName);

			m_FigureList.RemoveAt(pCurrent);

			delete pFig;

			return;
		}
	}*/
}

void CRVLGUI::Message(char *str,
					  int w, int h,
					  CvScalar color,
					  bool bWaitForKey)
{
	CRVLFigure *pFig = OpenFigure("Message");

	pFig->m_FontSize = 16;
	cvInitFont(&(pFig->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1);
	
	pFig->m_pImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

	pFig->m_Flags |= RVLFIG_FLAG_DATA;

	cvSet(pFig->m_pImage, color);

	cvPutText(pFig->m_pImage, str, cvPoint(50, (h + pFig->m_FontSize) / 2), &(pFig->m_Font), cvScalar(0, 0, 0));

	ShowFigure(pFig);

	if(bWaitForKey)
		cvWaitKey();

	CRVLGUI::CloseFigure("Message");
}

//*****************************************************
//
//     GLOBAL FUNCTIONS
//
//*****************************************************

// Conversion from HSL to RGB color format
// Source copied from http://www.geekymonkey.com/Programming/CSharp/RGB2HSL_HSL2RGB.htm
// h must be from the interval <-pi,pi] and l and s from [0, 1]

RVLCOLOR RVLHSL2RGB(double hrad, double s, double l)
{
	RVLCOLOR Color;

	double h = hrad / 2 / PI;

	if(h < 0.0)
		h += 1.0;

    double v;

    double r = l;   // default to gray

    double g = l;

    double b = l; 

    v = (l <= 0.5) ? (l * (1.0 + s)) : (l + s - l * s);

    if (v > 0) 

    {
          double m;

          double sv;

          int sextant;

          double fract, vsf, mid1, mid2;

          m = l + l - v;

          sv = (v - m ) / v;

          h *= 6.0;

          sextant = (int)h; 

          fract = h - sextant;

          vsf = v * sv * fract;

          mid1 = m + vsf;

          mid2 = v - vsf;

          switch (sextant) 
          {
                case 0: 

                      r = v; 

                      g = mid1; 

                      b = m; 

                      break;

                case 1: 

                      r = mid2; 

                      g = v; 

                      b = m; 

                      break;

                case 2: 

                      r = m; 

                      g = v; 

                      b = mid1; 

                      break;

                case 3: 

                      r = m; 

                      g = mid2; 

                      b = v; 

                      break;

                case 4: 

                      r = mid1; 

                      g = m; 

                      b = v; 

                      break;

                case 5: 

                      r = v; 

                      g = m; 

                      b = mid2; 

                      break;
          }

    }

	Color.r = (BYTE)(DOUBLE2INT(255.0 * r));
	Color.g = (BYTE)(DOUBLE2INT(255.0 * g));
	Color.b = (BYTE)(DOUBLE2INT(255.0 * b));

	return Color;
}


void RVLSaveDisplayVectors(	CRVLFigure *pFig,
							FILE *fp)
{
	CRVLDisplayVector *pVector;

	pFig->m_VectorArray.Start();

	while(pFig->m_VectorArray.m_pNext)
	{
		pVector = (CRVLDisplayVector *)(pFig->m_VectorArray.GetNext());

		RVLSaveDisplayVector(pVector, fp);
	}	
}

void RVLSaveDisplayVector(	CRVLDisplayVector *pVector,
							FILE *fp)
{
	if(pVector->m_PointArray.m_nElements == 0)
		return;

	pVector->m_PointArray.Start();

	RVLGUI_POINT *pP0 = (RVLGUI_POINT *)(pVector->m_PointArray.GetNext());	

	RVLGUI_POINT *pP1 = pP0;

	if(pVector->m_PointArray.m_pNext == NULL)
		return;

	RVLGUI_POINT *pP2;

	while(pVector->m_PointArray.m_pNext)
	{
		pP2 = (RVLGUI_POINT *)(pVector->m_PointArray.GetNext());

		fprintf(fp, "%lf\t%lf\t%lf\t%lf\n", (double)(pP1->u >> 1), (double)(pP1->v >> 1),
			(double)(pP2->u >> 1), (double)(pP2->v >> 1));

		pP1 = pP2;
	}

	if(pVector->m_bClosed)
		fprintf(fp, "%lf\t%lf\t%lf\t%lf\n", (double)(pP2->u >> 1), (double)(pP2->v >> 1),
			(double)(pP0->u >> 1), (double)(pP0->v >> 1));
}
/*
void RVLDisplay3DEllipse(double r1, 
						 double r2,
						 double maxErr,
						 CRVL3DPose *pPoseFC,
						 CRVLCamera *pCamera,
						 CvPoint **pPtArray,
						 int &n)
{
	n = 2 * DOUBLE2INT(PI / (2 * acos(1.0 - maxErr / r1))) + 1;

	CvPoint *PtArray = new CvPoint[n];

	*pPtArray = PtArray;

	double dgamma = 2.0 * PI / (double)n;

	double XF[3];

	XF[2] = 0.0;

	CvPoint *pPt = PtArray;

	double XC[3];
	double U[2];
	int iU[2];
	int i;
	double gamma;

	for(i = 0; i < n; i++, pPt++)
	{
		gamma = (double)i*dgamma;
		XF[0] = r1 * cos(gamma);
		XF[1] = r2 * sin(gamma);

		pPoseFC->Transf(XF, XC);

		pCamera->Project3DPoint(XC, U, iU);

		pPt->x = iU[0];
		pPt->y = iU[1];
	}	
}*/

#ifdef RVLOPENNI
void RVLTransformVectorsDepthToRGB(	CRVLFigure *pFig,
									short *D,
									int w,
									CRVLKinect *pKinect)
{
	if((pKinect->m_Flags & RVLKINECT_FLAG_DEVICE) == 0)
		return;

	openni::CoordinateConverter Converter;

	openni::VideoStream *pDepthStream = (openni::VideoStream *)(pKinect->m_vpDepthStream);
	openni::VideoStream *pColorStream = (openni::VideoStream *)(pKinect->m_vpColorStream);

	CRVLDisplayVector *pVector;
	RVLGUI_POINT *pPt;
	int u, v, uRGB, vRGB;
	short d;
	openni::DepthPixel z;

	pFig->m_VectorArray.Start();

	while(pFig->m_VectorArray.m_pNext)
	{
		pVector = (CRVLDisplayVector *)(pFig->m_VectorArray.GetNext());

		if(pVector->m_PointArray.m_nElements == 0)
			return;

		pVector->m_PointArray.Start();

		while(pVector->m_PointArray.m_pNext)
		{
			pPt = (RVLGUI_POINT *)(pVector->m_PointArray.GetNext());

			u = (pPt->u >> 1);
			v = (pPt->v >> 1);
			d = D[u + v * w];

			if(d != 2047)
			{
				z = (openni::DepthPixel)RVLKINECTDEPTHTOZ(d);

				Converter.convertDepthToColor(*pDepthStream, *pColorStream, u, v, z, &uRGB, &vRGB);

				pPt->u = (uRGB << 1) + 1;
				pPt->v = (vRGB << 1) + 1;
			}
		}
	}	
}
#endif

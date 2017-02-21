// RVLFigure.cpp: implementation of the CRVLFigure class.
//
//////////////////////////////////////////////////////////////////////

#include <highgui.h>
#include "RVLCore.h"
//#include "RVLObjectLib.h"
#include "RVLDisplayVector.h"
#include "RVLFigure.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLFigure::CRVLFigure()
{
	m_pImage = NULL;
	m_ImageName = NULL;
	m_State = RVLFIG_STATE_IDLE;
	m_ROIAnchorPt = cvPoint(0, 0);
	m_Flags = 0x00000000;
	m_pPixArray = NULL;
	m_ID = 0x00000000;
	m_iSelectedPix = -1;
	m_vpMouseCallbackData = NULL;
}

CRVLFigure::~CRVLFigure()
{
	if(m_ImageName)
		delete[] m_ImageName;

	if(m_Flags & RVLFIG_FLAG_DATA)
		cvReleaseImage(&m_pImage);

	if(m_Flags & RVLFIG_FLAG_MEM)
		delete m_pMem;
}


CRVLDisplayVector * CRVLFigure::AddVector(CRVLDisplayVector *pVectorTemplate)
{
	m_VectorArray.Add(pVectorTemplate);

	return (CRVLDisplayVector *)(m_VectorArray.GetLast());	
}



void CRVLFigure::Clear()
{
	//m_Flags = 0x00000000;
	m_pPixArray = NULL;
	m_VectorArray.RemoveAll();

	if(m_Flags & RVLFIG_FLAG_MEM)
		m_pMem->Clear();
}

void CRVLFigure::Create(CRVLMem *pMem)
{
	m_pMem = pMem;

	m_Flags &= ~RVLFIG_FLAG_MEM;

	m_VectorArray.Create(m_pMem, sizeof(CRVLDisplayVector));
}

void CRVLFigure::Create(int MemSize)
{
	m_pMem = new CRVLMem;

	m_pMem->Create(MemSize);

	m_Flags |= RVLFIG_FLAG_MEM;

	m_VectorArray.Create(m_pMem, sizeof(CRVLDisplayVector));
}

void CRVLFigure::DisplayFocus(CRVLCamera *pCamera)
{
	double X[3];

	//m_PoseC0.InvTransf(m_XFocus, X);

	double U[2];

	int iU[2];

	pCamera->Project3DPoint(X, U, iU);

	int size = 10;

	int u = (iU[0] >> 1);
	int v = (iU[1] >> 1);

	cvLine(m_pImage, cvPoint(u, v - size), cvPoint(u, v + size), cvScalar(0, 0, 255));
	cvLine(m_pImage, cvPoint(u - size, v), cvPoint(u + size, v), cvScalar(0, 0, 255));
}

BOOL CRVLFigure::MouseCallback3DDisplay(int event, int u, int v, int flags)
{
	CRVLGUI *pGUI = (CRVLGUI *)m_vpGUI;

	BOOL bDraw = FALSE;

	double du, dv;

	switch( event )
	{
		case CV_EVENT_LBUTTONUP:
			bDraw = TRUE;

			break;
		case CV_EVENT_MOUSEMOVE: 
			if(m_Flags & RVLFIG_FLAG_PREV_CURSOR_POSITION)
			{
				du = u - m_PrevCursorPosition.x;
				dv = v - m_PrevCursorPosition.y;
			}
			else
			{
				m_Flags |= RVLFIG_FLAG_PREV_CURSOR_POSITION;

				du = dv = 0;
			}

			//double *X = m_PoseC0.m_X;
			double *XF = m_XFocus;

			double R[3];

			//RVLDif3D(XF, X, R);

			double rxz = sqrt(R[0] * R[0] + R[2] * R[2]);

			double r = sqrt(RVLDotProduct(R, R));

			//double V[3];

			//V[0] = R[0] / r;
			//V[1] = R[1] / r;
			//V[2] = R[2] / r;

			double alpha = atan2(R[0], R[2]);

			double beta = (fabs(rxz) < APPROX_ZERO ? PI * (R[1] > 0.0 ? -1.0 : 1.0) : -atan(R[1] / rxz));
	
			if(flags & CV_EVENT_FLAG_LBUTTON)
			{
				// ego motion

				//pFig->m_PoseC0.m_Alpha -= (m_PoseC0ChangeScale.m_Alpha * du);

				//pFig->m_PoseC0.UpdateRotLL();

				//pFig->m_PoseC0.m_X[2] -= (m_PoseC0ChangeScale.m_X[0] * dv * cos(pFig->m_PoseC0.m_Alpha));
				//pFig->m_PoseC0.m_X[0] -= (m_PoseC0ChangeScale.m_X[0] * dv * sin(pFig->m_PoseC0.m_Alpha));

				// move around the focus

				//m_PoseC0.m_Alpha = alpha - (pGUI->m_PoseC0ChangeScale.m_Alpha * du);

				//m_PoseC0.m_Beta = beta - (pGUI->m_PoseC0ChangeScale.m_Alpha * dv);

				/*m_PoseC0.m_Theta = 0.0;

				m_PoseC0.UpdateRotLL();

				X[0] = XF[0] - r * m_PoseC0.m_Rot[3 * 0 + 2];
				X[1] = XF[1] - r * m_PoseC0.m_Rot[3 * 1 + 2];
				X[2] = XF[2] - r * m_PoseC0.m_Rot[3 * 2 + 2];*/

				/////

				bDraw = TRUE;
			}

			/*if(flags & CV_EVENT_FLAG_CTRLKEY)
			{
				double newrNrm = 1.0 - pGUI->m_PoseC0ChangeScale.m_X[0] * dv / r;

				X[0] = XF[0] - newrNrm * R[0];
				X[1] = XF[1] - newrNrm * R[1];
				X[2] = XF[2] - newrNrm * R[2];

				bDraw = TRUE;
			}*/

			m_PrevCursorPosition.x = u;
			m_PrevCursorPosition.y = v;

		//	break;
	
		//case CV_EVENT_RBUTTONUP:
		//				
		//	break;
		//case CV_EVENT_LBUTTONUP:
	}

	return bDraw;
}

void CRVLFigure::EmptyBitmap(CvSize size , CvScalar color)
{
	m_pImage = cvCreateImage(size, IPL_DEPTH_8U, 3);

	m_Flags |= RVLFIG_FLAG_DATA;

	cvSet(m_pImage, color);
}

void CRVLFigure::DisplayCovMx(	double *C,
								int u0, 
								int v0,
								CvScalar Color,
								double k)
{
	double VNrm[3 * 3];
	double eig[3];

	RVLMatrixHeaderA33->data.db = C;
	RVLMatrixHeaderB33->data.db = VNrm;
	RVLMatrixHeader31->data.db = eig;

	cvEigenVV(RVLMatrixHeaderA33, RVLMatrixHeaderB33, RVLMatrixHeader31);

	double r[3];

	r[0] = k * sqrt(fabs(eig[0]));
	r[1] = k * sqrt(fabs(eig[1]));
	r[2] = k * sqrt(fabs(eig[2]));

	char Text[201];

	sprintf(Text, "%lf %lf %lf", r[0], r[1], r[2]);

	PutText(Text, cvPoint(u0, v0 + m_FontSize), Color);

	sprintf(Text, "%lf %lf %lf", VNrm[0], VNrm[3], VNrm[6]);

	PutText(Text, cvPoint(u0, v0 + 2 * m_FontSize), Color);

	sprintf(Text, "%lf %lf %lf", VNrm[1], VNrm[4], VNrm[7]);

	PutText(Text, cvPoint(u0, v0 + 3 * m_FontSize), Color);

	sprintf(Text, "%lf %lf %lf", VNrm[2], VNrm[5], VNrm[8]);

	PutText(Text, cvPoint(u0, v0 + 4 * m_FontSize), Color);

}
void CRVLFigure::PutText(char * str, CvPoint position, CvScalar color)
{
	cvPutText(m_pImage, str, position, &m_Font, color);
}

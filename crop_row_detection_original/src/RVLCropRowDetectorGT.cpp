#include "highgui.h"
#include "RVLCore.h"
#include "RVLCropRowDetectorGT.h"

CRVLCropRowDetectorGT::CRVLCropRowDetectorGT(void)
{
	int i;

	for(i = 0; i < 2; i++)
	{
		m_Curve[i].n = 0;
		m_Curve[i].Segment = NULL;
	}

	m_c = NULL;
	m_d = NULL;

	m_InputImageFileName = NULL;

	m_E = 0;
}

CRVLCropRowDetectorGT::~CRVLCropRowDetectorGT(void)
{
	Clear();
}

void CRVLCropRowDetectorGT::Init()
{
	int w = m_pInputImage->width;
	int h = m_pInputImage->height;

	m_iCurve = 0;

	m_pSelectedSegment = NULL;

	Clear();

	int i;

	for(i = 0; i < 2; i++)
	{
		m_Curve[i].n = 0;

		m_Curve[i].Segment = new RVLCRD_GT_CURVE_SEGMENT[h];
	}

	m_c = new double[h];
	m_d = new double[h];

	m_pDisplay =  cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
}

void CRVLCropRowDetectorGT::Display()
{
	int PointWidth = 5;

	cvCopy(m_pInputImage, m_pDisplay);

	CvScalar Color, Color_;
	int iCurve;
	RVLCRD_GT_CURVE *pCurve;
	int iSegment;
	RVLCRD_GT_CURVE_SEGMENT *pSegment;
	int thickness;

	for(iCurve = 0; iCurve < 2; iCurve++)
	{
		pCurve = m_Curve + iCurve;

		Color = (iCurve == m_iCurve ? cvScalar(0, 255, 255) : cvScalar(0, 0, 255));

		for(iSegment = 0; iSegment < pCurve->n; iSegment++)
		{
			pSegment = pCurve->Segment + iSegment;

			if(pSegment == m_pSelectedSegment)
			{
				thickness = 2;
				Color_ = cvScalar(0, 128, 255);
			}
			else
			{
				thickness = 1;
				Color_ = Color;
			}

			cvLine(m_pDisplay, cvPoint((int)(pSegment->u) - PointWidth, pSegment->v), cvPoint((int)(pSegment->u) + PointWidth, pSegment->v), Color_, thickness);

			cvLine(m_pDisplay, cvPoint((int)(pSegment->u), pSegment->v - PointWidth), cvPoint((int)(pSegment->u), pSegment->v + PointWidth), Color_, thickness);
		}
	}

	if(m_Curve[0].n >= 2 && m_Curve[1].n >= 2)
	{
		int w = m_pInputImage->width;
		int h = m_pInputImage->height;

		double fw = (double)w;

		unsigned char *PixArray = (unsigned char *)(m_pDisplay->imageData);

		unsigned char *Pix;
		int u, v;
		double fu;

		for(v = m_v0; v < h; v++)
		{
			fu = m_c[v] - (floor(m_c[v] / m_d[v]) + 1.0) * m_d[v];

			while(fu < fw)
			{
				u = (int)fu;

				if(u >= 0 && u < w)
				{
					Pix = PixArray + 3 * (u + v * w);
					Pix[0] = 0;
					Pix[1] = 0;
					Pix[2] = 255;
				}

				fu += m_d[v];
			}
		}
	}

	CvFont Font;

	cvInitFont(&Font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1);

	char str[200];

	sprintf(str, "Curve %d", m_iCurve);

	cvPutText(m_pDisplay, str, cvPoint(0, 16), &Font,  cvScalar(0, 0, 255));

	cvShowImage("GT", m_pDisplay);

	cvSetMouseCallback("GT", RVLCRDGTMouseCallback, this);
}


void CRVLCropRowDetectorGT::Clear()
{
	cvReleaseImage(&m_pDisplay);

	if(m_Curve[0].Segment)
		delete[] m_Curve[0].Segment;

	if(m_Curve[1].Segment)
		delete[] m_Curve[1].Segment;

	if(m_c)
		delete[] m_c;

	if(m_d)
		delete[] m_d;
	
}
void CRVLCropRowDetectorGT::Edit(void)
{
	Init();

	Load();

	Display();

	int key = 0;

	do
	{
		key = cvWaitKey();

		switch(key){
		case 0x00250000:	// Left
			if(m_pSelectedSegment)
			{
				m_pSelectedSegment->u -= 0.1;

				ComputeCurve(m_iCurve);

				ComputeCRParams();
			}

			break;
		case 0x00270000:	// Right
			if(m_pSelectedSegment)
			{
				m_pSelectedSegment->u += 0.1;

				ComputeCurve(m_iCurve);

				ComputeCRParams();
			}

			break;
		case 0x002e0000:	// delete
			RemoveSegment();

			ComputeCurve(m_iCurve);

			ComputeCRParams();

			break;
		default:
			if(key != 27)
			{
				m_iCurve = 1 - m_iCurve;

				m_pSelectedSegment = NULL;
			}
		}

		Display();		
	}
	while(key != 27);

	cvDestroyWindow("GT");
}

void RVLCRDGTMouseCallback(int event, int x, int y, int flags, void* pData)
{
	CRVLCropRowDetectorGT *pGT = (CRVLCropRowDetectorGT *)pData;

	switch( event ){
	case CV_EVENT_LBUTTONDOWN:
		pGT->AddSegment(x, y);

		pGT->ComputeCurve(pGT->m_iCurve);

		pGT->ComputeCRParams();

		pGT->Display();

		break;
	case CV_EVENT_RBUTTONDOWN:
		pGT->SelectSegment(x, y);

		pGT->Display();
	}
}
void CRVLCropRowDetectorGT::AddSegment(int u, int v)
{
	RVLCRD_GT_CURVE *pCurve = m_Curve + m_iCurve;

	RVLCRD_GT_CURVE_SEGMENT *pSegment;

	int iSegment;

	for(iSegment = 0; iSegment < pCurve->n; iSegment++)
	{
		pSegment = pCurve->Segment + iSegment;

		if(v <= pSegment->v)
		{
			if(v < pSegment->v)
			{
				memmove(pSegment + 1, pSegment, (pCurve->n - iSegment) * sizeof(RVLCRD_GT_CURVE_SEGMENT));

				pCurve->n++;
			}

			break;
		}
	}

	if(iSegment == pCurve->n)
	{
		pSegment = pCurve->Segment + iSegment;

		pCurve->n++;
	}

	pSegment->u = (double)u;
	pSegment->v = v;

	m_pSelectedSegment = pSegment;
}

void CRVLCropRowDetectorGT::ComputeCurve(int iCurve)
{
	RVLCRD_GT_CURVE *pCurve = m_Curve + iCurve;

	if(pCurve->n < 2)
		return;

	RVLCRD_GT_CURVE_SEGMENT *pSegment, *pPrevSegment, *pNextSegment;
	int iSegment;
	double u1, v1, u2, v2, du, dv;		

	if(pCurve->n == 2)
	{
		pSegment = pCurve->Segment;

		u1 = pSegment->u;
		v1 = (double)(pSegment->v);
		
		pNextSegment = pSegment + 1;

		u2 = pNextSegment->u;
		v2 = (double)(pNextSegment->v);

		du = u2 - u1;
		dv = v2 - v1;

		pSegment->a0 = u1;
		pNextSegment->a0 = u2;
		pSegment->a1 = pNextSegment->a1 = du / dv;
		pSegment->a2 = pNextSegment->a2 = pSegment->a3 = pNextSegment->a3 = 0.0;
	}
	else if(pCurve->n >= 3)
	{
		// compute du/dv for segments 1, 2, ..., n - 2

		double V[2], V1[2], V2[2], fTmp;

		for(iSegment = 1; iSegment < pCurve->n - 1; iSegment++)
		{
			pSegment = pCurve->Segment + iSegment;

			pPrevSegment = pSegment - 1;

			pNextSegment = pSegment + 1;

			V1[0] = pNextSegment->u - pSegment->u;
			V1[1] = (double)(pNextSegment->v - pSegment->v);
			fTmp = sqrt(V1[0] * V1[0] + V1[1] * V1[1]);
			V1[0] /= fTmp;
			V1[1] /= fTmp;

			V2[0] = pSegment->u - pPrevSegment->u;
			V2[1] = (double)(pSegment->v - pPrevSegment->v);
			fTmp = sqrt(V2[0] * V2[0] + V2[1] * V2[1]);
			V2[0] /= fTmp;
			V2[1] /= fTmp;

			V[0] = V1[0] + V2[0];
			V[1] = V1[1] + V2[1];

			pSegment->du = V[0] / V[1];
		}

		// Segment 0 (parabola)

		pSegment = pCurve->Segment;

		u1 = pSegment->u;
		v1 = (double)(pSegment->v);
		
		pNextSegment = pSegment + 1;

		u2 = pNextSegment->u;
		v2 = (double)(pNextSegment->v);

		dv = v2 - v1;

		pSegment->a0 = u1;
		pSegment->a1 = 2.0 * (u2 - pSegment->a0) / dv - pNextSegment->du;
		pSegment->a2 = (pNextSegment->du - pSegment->a1) / (2.0 * dv);
		pSegment->a3 = 0.0;

		pSegment = pNextSegment;

		pNextSegment++;

		// Segments 1, 2, ..., n - 3 (cubic functions)

		for(iSegment = 1; iSegment < pCurve->n - 2; iSegment++, pNextSegment++)
		{
			u1 = u2;
			v1 = v2;

			u2 = pNextSegment->u;
			v2 = (double)(pNextSegment->v);

			du = u2 - u1;
			dv = v2 - v1;
			fTmp = du / dv;

			pSegment->a0 = u1;
			pSegment->a1 = pSegment->du;
			pSegment->a2 = (3.0 * fTmp - (2.0 * pSegment->du + pNextSegment->du)) / dv;
			pSegment->a3 = (-2.0 * fTmp + (pSegment->du + pNextSegment->du)) / (dv * dv);

			pSegment = pNextSegment;
		}

		// Segment n - 2 (parabola)	

		u1 = u2;
		v1 = v2;

		u2 = pNextSegment->u;
		v2 = (double)(pNextSegment->v);

		du = u2 - u1;
		dv = v2 - v1;

		pSegment->a0 = u1;
		pSegment->a1 = pSegment->du;
		pSegment->a2 = (du / dv - pSegment->du) / dv;
		pSegment->a3 = 0.0;

		pNextSegment->du = pSegment->a1 + 2.0 * pSegment->a2 * dv;
		
		// Segment n - 1 (line)

		pSegment = pNextSegment;

		u1 = pSegment->u;

		pSegment->a0 = u1;
		pSegment->a1 = pSegment->du;
		pSegment->a2 = pSegment->a3 = 0.0;
	}
}
void CRVLCropRowDetectorGT::ComputeCRParams(void)
{
	if(m_Curve[0].n < 2)
		return;

	if(m_Curve[1].n < 2)
		return;

	int h = m_pInputImage->height;

	RVLCRD_GT_CURVE_SEGMENT *pNextSegment1 = m_Curve[0].Segment;
	RVLCRD_GT_CURVE_SEGMENT *pNextSegment2 = m_Curve[1].Segment;

	m_v0 = RVLMAX(pNextSegment1->v, pNextSegment2->v);

	while(m_v0 >= pNextSegment1->v)
		pNextSegment1++;

	while(m_v0 >= pNextSegment2->v)
		pNextSegment2++;

	RVLCRD_GT_CURVE_SEGMENT *pSegment1 = pNextSegment1 - 1;
	RVLCRD_GT_CURVE_SEGMENT *pSegment2 = pNextSegment2 - 1;

	int v;
	double fv;

	for(v = m_v0; v < h; v++)
	{
		if(v == pNextSegment1->v)
		{
			pSegment1 = pNextSegment1;
			pNextSegment1++;
		}

		if(v == pNextSegment2->v)
		{
			pSegment2 = pNextSegment2;
			pNextSegment2++;
		}

		fv = (double)(v - pSegment1->v);

		m_c[v] = pSegment1->a0 + (pSegment1->a1 + (pSegment1->a2 + pSegment1->a3 * fv) * fv) * fv;

		fv = (double)(v - pSegment2->v);

		m_d[v] = pSegment2->a0 + (pSegment2->a1 + (pSegment2->a2 + pSegment2->a3 * fv) * fv) * fv - m_c[v];

		if(m_d[v] < 1.0)
			m_d[v] = 1.0;
	}
}

void CRVLCropRowDetectorGT::RemoveSegment()
{
	if(m_pSelectedSegment)
	{
		RVLCRD_GT_CURVE *pCurve = m_Curve + m_iCurve;
	
		pCurve->n--;

		if(pCurve->n > 0)
		{
			int iSegment = m_pSelectedSegment - pCurve->Segment;

			int nSegmentsToMove = pCurve->n - iSegment;

			if(nSegmentsToMove > 0)
				memmove(m_pSelectedSegment, m_pSelectedSegment + 1, nSegmentsToMove * sizeof(RVLCRD_GT_CURVE_SEGMENT));
		}

		m_pSelectedSegment = NULL;
	}
}

void CRVLCropRowDetectorGT::SelectSegment(int u, int v)
{
	int w = m_pInputImage->width;
	int h = m_pInputImage->height;

	RVLCRD_GT_CURVE *pCurve = m_Curve + m_iCurve;

	int mind2 = w * w + h * h;

	int du, dv, d2;
	int iSelectedSegment;
	RVLCRD_GT_CURVE_SEGMENT *pSegment;
	int iSegment;

	for(iSegment = 0; iSegment < pCurve->n; iSegment++)
	{
		pSegment = pCurve->Segment + iSegment;

		du = u - (int)(pSegment->u);
		dv = v - pSegment->v;
		d2 = du * du + dv * dv;

		if(d2 < mind2)
		{
			mind2 = d2;
			iSelectedSegment = iSegment;
		}
	}

	m_pSelectedSegment = pCurve->Segment + iSelectedSegment;
}

void CRVLCropRowDetectorGT::Save(void)
{
	FILE *fp = OpenFile("w");

	int iCurve, iSegment;
	RVLCRD_GT_CURVE *pCurve;
	RVLCRD_GT_CURVE_SEGMENT *pSegment;

	for(iCurve = 0; iCurve < 2; iCurve++)
	{
		pCurve = m_Curve + iCurve;

		fprintf(fp, "%d\n", pCurve->n);

		for(iSegment = 0; iSegment < pCurve->n; iSegment++)
		{
			pSegment = pCurve->Segment + iSegment;

			fprintf(fp, "%lf\t%d\n", pSegment->u, pSegment->v);
		}
	}

	fclose(fp);
}

bool CRVLCropRowDetectorGT::Load(void)
{
	FILE *fp = OpenFile("r");

	if(fp == NULL)
		return false;

	int iCurve, iSegment;
	RVLCRD_GT_CURVE *pCurve;
	RVLCRD_GT_CURVE_SEGMENT *pSegment;

	m_widthRatio = m_pInputImage->width / GTImageWidth;
	m_heightRatio = m_pInputImage->height / GTImageHeight;

	for(iCurve = 0; iCurve < 2; iCurve++)
	{
		pCurve = m_Curve + iCurve;

		fscanf(fp, "%d\n", &(pCurve->n));

		for(iSegment = 0; iSegment < pCurve->n; iSegment++)
		{
			pSegment = pCurve->Segment + iSegment;

			fscanf(fp, "%lf\t%d\n", &(pSegment->u), &(pSegment->v));
			
			pSegment->u *= m_widthRatio;
			pSegment->v *= m_heightRatio;
		}
	}

	fclose(fp);

	ComputeCurve(0);
	ComputeCurve(1);
	ComputeCRParams();

	return true;
}

void CRVLCropRowDetectorGT::SaveCRParams(void)
{
	int v;
	int h = m_pInputImage->height;;

	FILE *fp = OpenFile("w",".crp");

	//fprintf(fp, "%d\n", m_v0);

	for(v = m_v0; v < h; v++)
	{
		fprintf(fp, "%lf\t%lf\n", (m_c[v] - m_pInputImage->width/2), m_d[v]);
	}

	fclose(fp);
}

FILE * CRVLCropRowDetectorGT::OpenFile(char *mode, char *Extension)
{
	if(m_InputImageFileName == NULL)
		return NULL;

	char *GTFileName = RVLCreateString(m_InputImageFileName);

	//char Extension[] = ".gtr";

	strcpy(GTFileName + strlen(GTFileName) - strlen(Extension), Extension);

	FILE *fp = fopen(GTFileName, mode);

	delete[] GTFileName;

	return fp;
}

void CRVLCropRowDetectorGT::EvaluateParams(int *c, int *d, double sigma)
{
	m_E = 0;

	double E_iShift;
	int h = m_pInputImage->height;
	int w = m_pInputImage->width;
	int iShift, i;
	double u_GT;
	double u;
	int v;
	double distance;

	int m = 3;
	int start, end;

	start = -(m - 1) / 2;
	end = ((m - 1) / 2) + 1;

	for(iShift = -3; iShift < 4; iShift++)
	{

		E_iShift = 0;

		for(v = m_v0; v < h; v++)
		{
			for(i = start; i < end; i++)
			{
				//u_GT = (m_c[v] - w/2) + (iShift + i) * m_d[v];
				//u = c[v] + i * d[v];
				u_GT = m_c[v] + (iShift + i) * m_d[v];
				u = c[v] + w/2 + i * d[v];

				distance = ((u_GT - u)/(sigma * m_d[v])) * ((u_GT - u)/(sigma * m_d[v]));
				distance = 1 - distance;

				if(distance < 0)
					distance = 0;

				E_iShift += distance;
			}
		}
		
		E_iShift /= (m*(h - m_v0));
		
		if(E_iShift > m_E)
				m_E = E_iShift;
	}
}

void CRVLCropRowDetectorGT::EvaluateParams(double *u, double sigma)
{
	m_E = 0;

	double E_iShift;
	int h = m_pInputImage->height;
	int w = m_pInputImage->width;
	int iShift, i;
	double u_GT;
	double u_;
	int v;
	double distance;

	int m = 3;
	int start, end;

	start = -(m - 1) / 2;
	end = ((m - 1) / 2) + 1;

	for(iShift = -2; iShift < 3; iShift++)
	{

		E_iShift = 0;

		for(v = m_v0; v < h; v++)
		{
			for(i = start; i < end; i++)
			{
				u_GT = m_c[v] + (iShift + i) * m_d[v];
				u_ = u[v*m + i - start];

				distance = ((u_GT - u_)/(sigma * m_d[v])) * ((u_GT - u_)/(sigma * m_d[v]));
				distance = 1 - distance;

				if(distance < 0)
					distance = 0;

				E_iShift += distance;
			}
		}
		
		E_iShift /= (m*(h - m_v0));
		
		if(E_iShift > m_E)
				m_E = E_iShift;
	}
}

void CRVLCropRowDetectorGT::SaveImage(int experimentNum)
{
	char GTFileName[80];

	sprintf(GTFileName, "Results\\crop_row_GT_%d.bmp", experimentNum);

	cvSaveImage(GTFileName, m_pDisplay);
}

void CRVLCropRowDetectorGT::SaveUCoordinates(int *c, int *d)
{
	int h = m_pInputImage->height;
	int w = m_pInputImage->width;
	int i, v;
	double u;

	int m = 3;
	int start, end;

	start = -(m - 1) / 2;
	end = ((m - 1) / 2) + 1;

	FILE *fp = OpenFile("w",".tmg");

	for(v = 0; v < h; v++)
	{
		for(i = start; i < end; i++)
		{
			u = c[v] + w/2 + i * d[v];
			fprintf(fp, "%lf\t", u);
		}

		fprintf(fp, "\n");
	}

	fclose(fp);
}

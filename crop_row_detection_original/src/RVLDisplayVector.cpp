// RVLDisplayVector.cpp: implementation of the CRVLDisplayVector class.
//
//////////////////////////////////////////////////////////////////////

#include "RVLCore.h"
//#include "RVLObjectLib.h"
#include "RVLDisplayVector.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLDisplayVector::CRVLDisplayVector()
{
	m_bClosed = FALSE;
	m_LineWidth = 1;
	m_rL = 0;
	m_gL = 255;
	m_bL = 0;
	m_rP = 0;
	m_gP = 255;
	m_bP = 0;
	m_PointType = RVLGUI_POINT_DISPLAY_TYPE_SQUARE;
}

CRVLDisplayVector::CRVLDisplayVector(CRVLMem *pMem)
{
	m_bClosed = FALSE;
	m_LineWidth = 1;
	m_rL = 0;
	m_gL = 255;
	m_bL = 0;
	m_rP = 0;
	m_gP = 255;
	m_bP = 0;
	m_PointType = RVLGUI_POINT_DISPLAY_TYPE_SQUARE;

	m_PointArray.Create(pMem, sizeof(RVLGUI_POINT));	
}

CRVLDisplayVector::~CRVLDisplayVector()
{

}

void CRVLDisplayVector::Point(int u, int v)
{
	RVLGUI_POINT P;

	P.u = u;
	P.v = v;

	m_PointArray.Add(&P);
}


void CRVLDisplayVector::Line(int u1, int v1, int u2, int v2)
{
	Point(u1, v1);
	Point(u2, v2);
}


//void CRVLDisplayVector::Line(CRVL2DLine2 *p2DLine)
//{
//	Line(p2DLine->m_iU[0][0], p2DLine->m_iU[0][1], p2DLine->m_iU[1][0], p2DLine->m_iU[1][1]);
//}


void CRVLDisplayVector::Ellipse(double u0,
								double v0,
								double *C, 
								double scale, 
								double maxErr)
{
	int i;
	double gamma, p, q, fu, fv;
	double r1, r2;
	double phi;
	double InvC[4];

	InverseMatrix2(InvC, C, 1e-15);

	GetEllipseParams(InvC[0], InvC[1], InvC[3], r1, r2, phi);

	r1 *= scale;
	r2 *= scale;

	int n = DOUBLE2INT(PI / (2 * acos(1.0 - maxErr / r1))) + 1;

	double dgamma = PI / (double)n;

	double cs = cos(phi);
	double sn = sin(phi);

	for(i = 0; i < 2 * n; i++)
	{
		gamma = (double)i*dgamma;
		p = r1 * cos(gamma);
		q = r2 * sin(gamma);
		LinearTransform2D2(cs, sn, u0, v0, p, q, fu, fv);

		Point(DOUBLE2INT(2.0 * fu) + 1, DOUBLE2INT(2.0 * fv) + 1);
	}	
}




void CRVLDisplayVector::Rect(CvRect *pRect)
{
	Point(pRect->x, pRect->y);
	Point(pRect->x + pRect->width - 1, pRect->y);
	Point(pRect->x + pRect->width - 1, pRect->y + pRect->height - 1);
	Point(pRect->x, pRect->y + pRect->height - 1);
}

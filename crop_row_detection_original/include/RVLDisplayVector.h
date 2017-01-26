// RVLDisplayVector.h: interface for the CRVLDisplayVector class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLDISPLAYVECTOR_H__BC66C7F7_52BE_4746_BCDC_399FA65F9F76__INCLUDED_)
#define AFX_RVLDISPLAYVECTOR_H__BC66C7F7_52BE_4746_BCDC_399FA65F9F76__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define RVLGUI_POINT_DISPLAY_TYPE_NONE 		0
#define RVLGUI_POINT_DISPLAY_TYPE_SQUARE	1
#define RVLGUI_POINT_DISPLAY_TYPE_DOT		2
#define RVLGUI_POINT_DISPLAY_TYPE_CROSS		3

struct RVLGUI_POINT
{
	int u, v;
};

class CRVLDisplayVector  
{
public:
	BYTE m_rL, m_gL, m_bL;
	BYTE m_rP, m_gP, m_bP;
	int m_LineWidth;
	BYTE m_PointType;
	BOOL m_bClosed;
	CRVLMChain m_PointArray;

public:
	void Rect(CvRect *pRect);
	void Ellipse(double u0,
				 double v0,
				 double *C, 
				 double scale, 
				 double maxErr);
	//void Line(CRVL2DLine2 *p2DLine);
	CRVLDisplayVector(CRVLMem *pMem);
	void Line(int u1, int v1, int u2, int v2);
	void Point(int u, int v);
	CRVLDisplayVector();
	virtual ~CRVLDisplayVector();
};

#endif // !defined(AFX_RVLDISPLAYVECTOR_H__BC66C7F7_52BE_4746_BCDC_399FA65F9F76__INCLUDED_)

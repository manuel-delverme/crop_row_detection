// RVLFigure.h: interface for the CRVLFigure class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLFIGURE_H__A3549230_F133_49DC_9FDF_BC4ADF1CAF35__INCLUDED_)
#define AFX_RVLFIGURE_H__A3549230_F133_49DC_9FDF_BC4ADF1CAF35__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define RVLFIG_FLAG_DATA					0x01000000	// bits 0-23 are set by SAS
#define RVLFIG_FLAG_PREV_CURSOR_POSITION	0x02000000
#define RVLFIG_FLAG_MEM						0x04000000
#define RVLFIG_FLAG_RESET					0x08000000
#define RVLFIG_FLAG_VECTORS_LOCKED			0x10000000
#define RVLFIG_FLAG_ALTERNATIVE_SELECTION	0x20000000
#define RVLFIG_STATE_IDLE			0
#define RVLFIG_STATE_DEFROI			1
#define RVLFIG_STATE_DRAGROI		2
#define RVLFIG_ID_HOG				1

class CRVLFigure  
{
public:
	CRVLMem *m_pMem;
	DWORD m_Flags;
	int m_u0, m_v0;
	PIX_ARRAY *m_pPixArray;
	CRVLMChain m_VectorArray;
	IplImage *m_pImage;
	CvRect m_ROI;
	CvPoint m_ROIAnchorPt;
	char *m_ImageName;
	BYTE m_State;
	DWORD m_ID;
	void *m_vpGUI;
	void *m_vpMouseCallbackData;
	CvPoint m_PrevCursorPosition;
	//CRVL3DPose m_PoseC0;
	double m_XFocus[3];
	int m_iSelectedPix;
	CvFont m_Font;
	int m_FontSize;

public:
	void DisplayCovMx(	double *C,
						int u0, 
						int v0,
						CvScalar Color,
						double k = 1.0);
	BOOL MouseCallback3DDisplay(int event, int u, int v, int flags);
	void DisplayFocus(CRVLCamera *pCamera);
	void Create(CRVLMem *pMem);
	void Clear();
	CRVLDisplayVector * AddVector(CRVLDisplayVector *pVectorTemplate);
	CRVLFigure();
	virtual ~CRVLFigure();

	void Create(int MemSize);
	void EmptyBitmap(CvSize size , CvScalar color);
	void PutText(char * str, CvPoint position, CvScalar color);
};

#endif // !defined(AFX_RVLFIGURE_H__A3549230_F133_49DC_9FDF_BC4ADF1CAF35__INCLUDED_)

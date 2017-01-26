// RVLGUI.h: interface for the CRVLGUI class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLGUI_H__67C03517_A5D9_4A71_914C_608ADB97A9C6__INCLUDED_)
#define AFX_RVLGUI_H__67C03517_A5D9_4A71_914C_608ADB97A9C6__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RVLDisplayVector.h"
#include "RVLFigure.h"
//#include "RVLDataPlot.h"
//#include "RVLMutex.h"

#define RVLGUI_COMMAND_KEY_F1					1
#define RVLGUI_COMMAND_KEY_F2					2
#define RVLGUI_COMMAND_KEY_F3					3
#define RVLGUI_COMMAND_KEY_F4					4
#define RVLGUI_COMMAND_KEY_F5					5
#define RVLGUI_COMMAND_KEY_F6					6
#define RVLGUI_COMMAND_KEY_F7					7
#define RVLGUI_COMMAND_KEY_F8					8
#define RVLGUI_COMMAND_KEY_F9					9
#define RVLGUI_COMMAND_KEY_F10					10
#define RVLGUI_COMMAND_KEY_F11					11
#define RVLGUI_COMMAND_KEY_F12					12
#define RVLGUI_COMMAND_KEY_HOME					13
#define RVLGUI_COMMAND_KEY_END					14
#define RVLGUI_COMMAND_KEY_PGUP					15
#define RVLGUI_COMMAND_KEY_PGDN					16
#define RVLGUI_COMMAND_KEY_UP					17
#define RVLGUI_COMMAND_KEY_DOWN					18
#define RVLGUI_COMMAND_KEY_LEFT					19
#define RVLGUI_COMMAND_KEY_RIGHT				20
#define RVLGUI_COMMAND_MOUSE_LBUTTON_PRESSED	21
#define RVLGUI_COMMAND_MOUSE_LBUTTON_RELEASED	22
#define RVLGUI_COMMAND_MOUSE_RBUTTON_PRESSED	23
#define RVLGUI_COMMAND_MOUSE_RBUTTON_RELEASED	24
//#define RVLGUI_PARAM_ID_SAMPLE_DIR			0
//#define RVLGUI_PARAM_ID_INPUT_IMAGE			1
//#define RVLGUI_PARAM_ID_DISPLAY1				2
//#define RVLGUI_PARAM_ID_DISPLAY2				3

#define RVLGUI_FLAG_INPUT_IMAGE					0x00000001
#define RVLGUI_FLAG_STRETCH						0x00000002
#define RVLGUI_FLAG_ALTERNATIVE_NEXT_STEP		0x00000004
#define RVLGUI_FLAG_ALTERNATIVE_PREV_STEP		0x00000008
#define RVLGUI_FLAG_WAIT						0x00000010
#define RVLGUI_INIT_RES_ERR_CFG_FILE			0x00000001
#define RVLGUI_MAXNTEXTLINES					10
#define RVLGUI_OPTICAL_FLOW_DISPLAY_COLOR_LT_HALF_SIZE		100
#define RVLGUI_SELECTED_OBJECT_TYPE_UNDEFINED	0
#define RVLGUI_SELECTED_OBJECT_TYPE_SURFACE		1
#define RVLGUI_SELECTED_OBJECT_TYPE_LINE		2

struct RVLCOLOR
{
	BYTE r, g, b;
};

class CRVLGUI;

struct RVLGUI_MOUSE_CALLBACK_DATA
{
	CRVLFigure *pFig;
	CRVLGUI *pGUI;
};

inline RVLCOLOR RVLColor(BYTE r, BYTE g, BYTE b)
{
	RVLCOLOR Color;

	Color.r = r;
	Color.g = g;
	Color.b = b;

	return Color;
};

RVLCOLOR RVLHSL2RGB(double h, double s, double l);
void RVLSaveDisplayVectors(	CRVLFigure *pFig,
							FILE *fp);
void RVLSaveDisplayVector(	CRVLDisplayVector *pVector,
							FILE *fp);
/*void RVLDisplay3DEllipse(double r1, 
						 double r2,
						 double maxErr,
						 CRVL3DPose *pPoseFC,
						 CRVLCamera *pCamera,
						 CvPoint **pPtArray,
						 int &n);*/
#ifdef RVLOPENNI
void RVLTransformVectorsDepthToRGB(	CRVLFigure *pFig,
									short *D,
									int w,
									CRVLKinect *pKinect);
#endif

class CRVLGUI  
{
public:
	DWORD m_Flags;
	DWORD m_Command;
	CRVLMem *m_pMem0;
	CRVLMem *m_pMem;
	CRVLFigure m_Figure[2];
	//CRVLMPtrChain m_FigureList;
	//char m_ParamFileName[RVL_MAX_FILE_NAME_LEN];
	char *m_SampleDirectory;
	char *m_OutputDirectory;
	char *m_InputImageFileName;
	//char m_InputSampleFileName[RVL_MAX_FILE_NAME_LEN];
	//char m_InputImageUVFileName[RVL_MAX_FILE_NAME_LEN];
	//char m_OutputImageFileName[RVL_MAX_FILE_NAME_LEN];
	CRVLParameterList m_ParamList;	
	int m_nSequenceSamples;
	DWORD m_SelectedObjectType;
	void *m_pSelectedObject;
	void *m_pSelectedObject2;
	int m_iStep;
	int m_nSteps;
	int m_nOutImgVers;
	int m_OutImgVer;
	char m_Text[RVLGUI_MAXNTEXTLINES][200];
	int m_nTextLines;
	RVLCOLOR m_OpticalFlowDisplayColorLT[
		(2 * RVLGUI_OPTICAL_FLOW_DISPLAY_COLOR_LT_HALF_SIZE + 1) * 
		(2 * RVLGUI_OPTICAL_FLOW_DISPLAY_COLOR_LT_HALF_SIZE + 1)];
	//CRVL3DPose m_PoseC0ChangeScale;
	//CRVLMutex *m_pMutex;

public:
	CRVLGUI();
	virtual ~CRVLGUI();
	CRVLFigure *OpenFigure(	char *ImageName,
							int MemSize = 1000000);
	void ShowFigure(CRVLFigure *pFig);
	void DisplayVectors(CRVLFigure *pFig,
						int u0, int v0,
						double ZoomFactor);
	void DisplayVector(	CRVLDisplayVector *pVector,
						IplImage *pImage,
						int u0, int v0,
						double ZoomFactor);
	CRVLFigure *GetFigure(DWORD ID);
	void ClearDisplay();
	void CreateParamList();
	void Clear();
	DWORD Init();
	DWORD Init(char *CfgFileName);
	void CloseFigure(char * ImageName);
	void Message(	char *str,
					int w, int h,
					CvScalar color,
					bool bWaitForKey = true);

};

#endif // !defined(AFX_RVLGUI_H__67C03517_A5D9_4A71_914C_608ADB97A9C6__INCLUDED_)

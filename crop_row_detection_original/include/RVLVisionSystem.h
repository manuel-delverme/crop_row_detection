#pragma once

#define RVLVS_ERR_CAMERA		0x01000000
#define RVLVS_ERR_EDGE_MAP_FILE	0x02000000

#define RVLSYS_FLAGS_IMAGE						0x00000001
#define RVLSYS_FLAGS_WARP_IMAGE					0x00000002
#define RVLSYS_FLAGS_STEREO						0x00000004

#define RVLSYS_FLAGS2_STEREO_ROI				0x00000001

#define RVLSYS_MCMEMSIZE		2

//#define RVLSYS_PSULMBRLM_UPDATE_LOG_FILE

class CRVLVisionSystem
{
public:
	DWORD m_Flags, m_Flags2;
	CRVLMem m_Mem0, m_Mem, m_Mem2, m_MCMem[RVLSYS_MCMEMSIZE];
	int m_iMCMem;
	CRVLCamera m_CameraL, m_CameraR;
	CRVLAImage m_AImage;
	//RVLRECT m_ROI;
	CRVLStereoVision m_StereoVision;
	//CRVLTimer *m_pTimer;
	char *m_ImageFileName;
	//CRVLKinect m_Kinect;
	int m_Mem0Size;
	int m_MemSize;
	int m_Mem2Size;
	int m_MCMemSize;
	CRVLParameterList m_ParamList;
	CRVLMem m_ParamMem;

public:
	CRVLVisionSystem();
	virtual ~CRVLVisionSystem();
	DWORD Init(char *CfgFile2Name = NULL);
	void Clear();
	void UpdateMem();
	virtual BOOL RealTimeDisplay(CRVLGUI *pGUI);
	virtual void CreateParamList();
};

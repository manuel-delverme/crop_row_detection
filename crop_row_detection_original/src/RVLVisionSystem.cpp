#include "highgui.h"
#include "RVLCore.h"

CRVLVisionSystem::CRVLVisionSystem()
{
	m_Flags = m_Flags2 = 0x00000000;

	//m_AImage.m_pPix = NULL;

	//m_pTimer = NULL;

	m_ImageFileName = NULL;

	m_Mem0Size = 150000000;
	m_MemSize = 150000000;
	m_Mem2Size = 150000000;
	m_MCMemSize = 10000000;
}

CRVLVisionSystem::~CRVLVisionSystem()
{
	Clear();

	/*if(m_pTimer)
		delete m_pTimer;*/

	if(m_ImageFileName)
		delete[] m_ImageFileName;
}

void CRVLVisionSystem::CreateParamList()
{
	m_ParamMem.Create(1000000);

	m_ParamList.m_pMem = &m_ParamMem;

	RVLPARAM_DATA *pParamData;

	m_ParamList.Init();

	pParamData = m_ParamList.AddParam("VS.Mem0Size", RVLPARAM_TYPE_INT, &m_Mem0Size);
	pParamData = m_ParamList.AddParam("VS.MemSize", RVLPARAM_TYPE_INT, &m_MemSize);
	pParamData = m_ParamList.AddParam("VS.Mem2Size", RVLPARAM_TYPE_INT, &m_Mem2Size);
	pParamData = m_ParamList.AddParam("VS.MCMemSize", RVLPARAM_TYPE_INT, &m_MCMemSize);
	pParamData = m_ParamList.AddParam("VS.ImageFileName", RVLPARAM_TYPE_STRING, &m_ImageFileName);
}

DWORD CRVLVisionSystem::Init(char *CfgFile2Name)
{
	rtUtilInit();

	// load parameters

	m_ParamList.LoadParams(CfgFile2Name);

	// allocate memory

	m_Mem0.Create(m_Mem0Size);

	m_Mem.Create(m_MemSize);

	m_Mem2.Create(m_Mem2Size);

	m_MCMem[0].Create(m_MCMemSize);
	m_MCMem[1].Create(m_MCMemSize);

	m_iMCMem = 0;

	// initialize stereo vision system

	//m_StereoVision.m_Flags = RVLSTEREO_FLAGS_CALIB_METHOD_SVS;

	m_StereoVision.m_pMem = &m_Mem;
	m_StereoVision.m_pMem2 = &m_Mem2;
	m_StereoVision.m_pCameraL = &m_CameraL;
	m_StereoVision.m_pCameraR = &m_CameraR;

	m_StereoVision.CreateParamList(&m_Mem0);

	if(CfgFile2Name)
		m_StereoVision.m_ParamList.LoadParams(CfgFile2Name);

	//if(m_Flags & RVLSYS_FLAGS_STEREO)
		//if((m_StereoVision.m_Flags & RVLSTEREO_FLAGS_METHOD_SVS) == 0)
		//	m_StereoVision.CreateTriangulationLookupTable();

	if(m_StereoVision.m_Flags & RVLSTEREO_FLAGS_CALIB_METHOD_SVS)
		m_StereoVision.InitCamerasSVS(m_StereoVision.m_ParamFileName);
	//else
	//{
	//	DWORD res;

	//	res = m_CameraL.Init(m_StereoVision.m_ParamFileName);

	//	if(res != RVL_RES_OK)
	//		return RVLVS_ERR_CAMERA | res;

	//	res = m_CameraR.Init(m_StereoVision.m_ParamFileName);	// this should be changed if the right camera is different
	//								
	//	if(res != RVL_RES_OK)
	//		return RVLVS_ERR_CAMERA | res;

	//	m_CameraL.GetParallelStereoCameraSystem(&m_CameraR);
	//
	//	m_CameraL.CreateImageNrmLookupTable(0);
	//	m_CameraR.CreateImageNrmLookupTable(0);
	//}

	m_StereoVision.Init();

	//m_Kinect.m_scale = m_StereoVision.m_KinectScale;

	// initialize ROI
/*
	if(m_Flags2 & RVLSYS_FLAGS2_STEREO_ROI)
	{
		m_ROI.left = m_StereoVision.m_nDisp;
		m_ROI.right = m_CameraL.Width - 1;
		m_ROI.top = 0;
		m_ROI.bottom = m_CameraL.Height - 1;

		m_AImage.m_pROI = &m_ROI;
	}*/

	// initialize A-image

	m_AImage.m_pMem = &m_Mem;
	m_AImage.m_pMem2 = &m_Mem2;
	m_AImage.m_pCameraL = &m_CameraL;
	m_AImage.m_pCameraR = &m_CameraR;
	m_AImage.m_bOwnData = TRUE;

	/*if(m_AImage.m_pPix != NULL)
		delete[] m_AImage.m_pPix;

	m_AImage.m_pPix = (RVLAPIX *)malloc(m_CameraL.Width * m_CameraL.Height * 
		sizeof(RVLAPIX));*/	

	m_AImage.Init();

	// timer

	/*if(m_pTimer == NULL)
		m_pTimer = new CRVLTimer;*/

	return RVL_RES_OK;
}

void CRVLVisionSystem::Clear()
{
	//m_AImage.Clear();
}

void CRVLVisionSystem::UpdateMem()
{
	m_Mem.Clear();

	m_iMCMem = (m_iMCMem + 1) % RVLSYS_MCMEMSIZE;

	m_MCMem[m_iMCMem].Clear();			
}

BOOL CRVLVisionSystem::RealTimeDisplay(CRVLGUI *pGUI)
{
	return FALSE;
}

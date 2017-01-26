// RVLClass.h: interface for the CRVLClass class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLCLASS_H__264C6694_60FB_4501_889C_62C505E7AE94__INCLUDED_)
#define AFX_RVLCLASS_H__264C6694_60FB_4501_889C_62C505E7AE94__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define RVLRELLIST_TYPE_NONE			0x00
#define RVLRELLIST_TYPE_CHAIN			0x01
#define RVLRELLIST_TYPE_ARRAY			0x02
#define RVLRELLIST_TYPE_ORDERED			0x04
#define RVLRELLIST_MS_MATCH				0
#define RVLRELLIST_UNCERT_BOUND			1
#define RVLRELLIST_NEIGHBORS			2
#define RVLRELLIST_COMPONENTS			3
#define RVLRELLIST_ELEMENTS				4
#define RVLRELLIST_CONTOURS				5
#define RVLOBJ2_DATA_MATCH_PTR			0
#define RVLOBJ2_DATA_NEW_X				1
#define RVLOBJ2_DATA_TEMP				2
#define RVLOBJ2_DATA_DESCRIPTOR			3
#define RVLOBJ2_DATA_VIEW				4
#define RVLOBJ2_DATA_PARENT_PTR			5
#define RVLOBJ2_DATA_PROJECT_PTR		6
#define RVLOBJ2_DATA_GRANDPARENT_PTR	7
#define RVLOBJ2_DATA_NODE_PTR			8
#define RVLOBJ2_DATA_CONTOUR_PTR		9
//#define RVLOBJ2_DATA_LAST_VIEW_PTR	5
#define RVLOBJ2_DATA_SEGMENT_ARRAY		5
#define RVLCLS_FLAG_FULL				0x00000001
#define RVLCLS_FLAG_PERMANENT			0x00000002
#define RVLCLS_FLAG_ARRAY				0x00000004
#define RVLCLS_FLAG_LIST				0x00000008
#define RVLCLS_FLAG_MULTIVIEW			0x00000010

#define RVLCLS_TYPE_UNDEFINED			0
#define RVLCLS_TYPE_3DOBJECT3			1
#define RVLCLS_TYPE_3DPOINT3			2
#define RVLCLS_TYPE_3DOBJECT2			3
#define RVLCLS_TYPE_2DOBJECT2			4
#define RVLCLS_TYPE_2DPOINT3			5
#define RVLCLS_TYPE_2DSIFT				6


struct RVLRELLIST_DESC
{
	int index;
	BYTE type;
};

class CRVLClass  
{
public:	
	WORD m_Type;
	int m_DataSize;
	DWORD m_ID;
	DWORD m_Flags;
	int m_nObjects;
	int m_nRelLists;
	RVLRELLIST_DESC *m_RelListDesc;
	CRVLMem *m_pMem0, *m_pMem, *m_pMem2;
	//CRVLMPtrChain m_ObjectList;
	void **m_ObjectArray;
	//CRVLMPtrChain *m_pRelList;
	int m_iRelListNeighbors;
	int m_iRelListComponents;
	int m_iRelListElements;
	int m_iRelListSuperObjects;
	int m_iRelListMSMatch;
	int m_iRelListMProject;
	int m_iRelListUncertBound;
	int m_iRelListContours;
	int m_iDataDisparity;				// [VAR 80]
	int m_iDataProjectPtr;				// [VAR 81]
	int m_iData3DObjectPtr;				// [VAR 82]
	int m_iDataFit3DPointsWithUncert;
	int m_iData3DPoint;
	int m_iData3DPoint2;
	int m_iDataXRot;
	int m_iDataXTransf;
	int m_iDataWeight;
	int m_iDataScale;
	int m_iData2DModelPtr;
	int m_iDataMatchPtr;
	int m_iDataR;
	int m_iDataRotJacobian;
	int m_iDataRotCov;
	int m_iDataMSCov;
	int m_iDataUncertTransfCov;
	int m_iDataXNew;
	int m_iDataTemp;
	int m_iDataDescriptor;
	int m_iDataView;
	int m_iDataParentPtr;
	int m_iDataGrandParentPtr;
	int m_iDataNodePtr;
	//int m_iDataLastViewPtr;
	int m_iDataSegmentArray;
	int m_iDataContourPtr;
	int m_DescriptorSize;
	double m_maxMatchDist;
	int m_maxMatchQuality;
	double m_fmaxMatchQuality;
	double m_varMatchQuality;
	int m_maxMatchCost;
	double m_MatchCostNrm;
	int m_DescriptorMatchTol;
	//CRVLMPtrChain2 m_RelList;
	int m_iData[5];
	int m_iRelList[4];
	int m_nPermanentObjectPlaces;
	void **m_PermanentObjectArray;
	void **m_pFirstFreeObjectPlace;
	//int m_nPermanentViewPlaces;
	//void **m_PermanentViewArray;
	//void **m_pFirstFreeViewPlace;
	//CRVLTimer *m_pTimer;
	//CvMat *m_mxDescriptor;
	int m_SIFTDescriptorLength; // Length of SIFT descriptor
	int m_HistRGBBaseLog2;
	double m_UncertCoeff;
	double m_OverlapCoeff;

public:
	void AppendData(int DataID, int DataSize = -1);
	void Clear();
	void Add(void *pObject);
	void Update();
	void Init();
	CRVLClass();
	virtual ~CRVLClass();
};

#endif // !defined(AFX_RVLCLASS_H__264C6694_60FB_4501_889C_62C505E7AE94__INCLUDED_)

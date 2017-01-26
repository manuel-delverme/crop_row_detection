// RVLClass.cpp: implementation of the CRVLClass class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "RVLCore.h"
#include "RVLClass.h"

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

#define RVLCLASS_ID_2D			0x00000001

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLClass::CRVLClass()
{
	m_Flags = RVLCLS_FLAG_LIST;
	m_RelListDesc = NULL;
	m_nRelLists = 0;
	m_DataSize = 0;
	m_nObjects = 0;
	m_iDataFit3DPointsWithUncert = -1;
	m_iData2DModelPtr = -1;
	m_iData3DObjectPtr = -1;
	m_iData3DPoint = -1;
	m_iData3DPoint2 = -1;
	m_iDataWeight = -1;
	m_iDataProjectPtr = -1;
	m_iDataScale = -1;
	m_iDataMatchPtr = -1;
	m_iDataR = -1;
	m_iDataDisparity = -1;
	m_iDataRotJacobian = -1;
	m_iDataRotCov = -1;
	m_iDataMSCov = -1;
	m_iDataXNew = -1;
	m_iDataTemp = -1;
	m_iDataXRot = -1;
	m_iDataXTransf = -1;
	m_iDataUncertTransfCov = -1;
	m_iDataDescriptor = -1;
	m_iDataView = -1;
	m_iDataSegmentArray = -1;
	m_iDataParentPtr = -1;
	m_iDataGrandParentPtr = -1;
	m_iDataNodePtr = -1;
	m_iRelListNeighbors = -1;
	m_iRelListComponents = -1;
	m_iRelListSuperObjects = -1;
	m_iRelListMSMatch = -1;
	m_iRelListUncertBound = -1;
	m_iRelListContours = -1;
	//m_iDataLastViewPtr = -1;
	m_PermanentObjectArray = NULL;
	//m_mxDescriptor = NULL;
	//m_DescriptorMatchTol = 125;			// default
	m_DescriptorMatchTol = 100;
	m_HistRGBBaseLog2 = 4;
	m_UncertCoeff = 1.0;
	m_OverlapCoeff = 1.0;
}


CRVLClass::~CRVLClass()
{
	if(m_RelListDesc != NULL)
		delete[] m_RelListDesc;

	//if(m_mxDescriptor)
	//	cvReleaseMat(&m_mxDescriptor);
}


void CRVLClass::Init()
{
	//m_ObjectList.m_pMem = m_pMem0;

	Update();

	if(m_Flags & RVLCLS_FLAG_PERMANENT)
	{
		int Size = m_nPermanentObjectPlaces * sizeof(void *);

		m_PermanentObjectArray = (void **)(m_pMem0->Alloc(Size));

		memset(m_PermanentObjectArray, 0, Size);

		m_pFirstFreeObjectPlace = m_PermanentObjectArray;
	}
}


void CRVLClass::Update()
{
	m_iData[RVLOBJ2_DATA_MATCH_PTR] = m_iDataMatchPtr;
	m_iData[RVLOBJ2_DATA_NEW_X] = m_iDataXNew;
	m_iData[RVLOBJ2_DATA_TEMP] = m_iDataTemp;
	m_iData[RVLOBJ2_DATA_DESCRIPTOR] = m_iDataDescriptor;
	m_iData[RVLOBJ2_DATA_VIEW] = m_iDataView;
	//m_iData[RVLOBJ2_DATA_LAST_VIEW_PTR] = m_iDataLastViewPtr;
	m_iData[RVLOBJ2_DATA_SEGMENT_ARRAY] = m_iDataSegmentArray;
	m_iData[RVLOBJ2_DATA_PARENT_PTR] = m_iDataParentPtr;
	m_iData[RVLOBJ2_DATA_GRANDPARENT_PTR] = m_iDataGrandParentPtr;
	m_iData[RVLOBJ2_DATA_NODE_PTR] = m_iDataNodePtr;
	m_iData[RVLOBJ2_DATA_CONTOUR_PTR] = m_iDataContourPtr;

	m_iRelList[RVLRELLIST_MS_MATCH] = m_iRelListMSMatch;
	m_iRelList[RVLRELLIST_NEIGHBORS] = m_iRelListNeighbors;
	m_iRelList[RVLRELLIST_COMPONENTS] = m_iRelListComponents;
	m_iRelList[RVLRELLIST_ELEMENTS] = m_iRelListElements;
	m_iRelList[RVLRELLIST_CONTOURS] = m_iRelListContours;
}


void CRVLClass::Add(void *pObject)
{
	if(m_Flags & RVLCLS_FLAG_ARRAY)
		m_ObjectArray[m_nObjects++] = pObject;
		
	/*if(m_Flags & RVLCLS_FLAG_LIST)
		m_ObjectList.Add(pObject);	*/		
}

void CRVLClass::Clear()
{
	/*if(m_Flags & RVLCLS_FLAG_LIST)
		m_ObjectList.RemoveAll();*/

	m_nObjects = 0;
}

void CRVLClass::AppendData(int DataID, 
						   int DataSize)
{
	switch(DataID){
	case RVLOBJ2_DATA_NEW_X:
		m_iDataXNew = m_DataSize;
		m_DataSize += (3 * sizeof(double));
		m_iData[RVLOBJ2_DATA_NEW_X] = m_iDataXNew;
		break;
	case RVLOBJ2_DATA_PARENT_PTR:
		m_iDataParentPtr = m_DataSize;
		m_DataSize += sizeof(void *);
		m_iData[RVLOBJ2_DATA_PARENT_PTR] = m_iDataParentPtr;
		break;
	case RVLOBJ2_DATA_GRANDPARENT_PTR:
		m_iDataGrandParentPtr = m_DataSize;
		m_DataSize += sizeof(void *);
		m_iData[RVLOBJ2_DATA_GRANDPARENT_PTR] = m_iDataGrandParentPtr;
		break;
	case RVLOBJ2_DATA_PROJECT_PTR:
		m_iDataProjectPtr = m_DataSize;
		m_DataSize += sizeof(void *);
		m_iData[RVLOBJ2_DATA_PROJECT_PTR] = m_iDataProjectPtr;
		break;
	case RVLOBJ2_DATA_NODE_PTR:
		m_iDataNodePtr = m_DataSize;
		m_DataSize += sizeof(void *);
		m_iData[RVLOBJ2_DATA_NODE_PTR] = m_iDataNodePtr;
		break;
	case RVLOBJ2_DATA_CONTOUR_PTR:
		m_iDataContourPtr = m_DataSize;
		m_DataSize += sizeof(void *);
		m_iData[RVLOBJ2_DATA_CONTOUR_PTR] = m_iDataContourPtr;
	}
}


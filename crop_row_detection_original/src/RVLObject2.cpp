// RVLObject2.cpp: implementation of the CRVLObject2 class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "RVLCore.h"
#include "RVLClass.h"
#include "RVLObject2.h"

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLObject2::CRVLObject2()
{
	m_Flags = 0x00000000;
	m_ParamFlags = 0x00000000;
}

CRVLObject2::~CRVLObject2()
{

}

void CRVLObject2::Create(CRVLClass *pClass)
{
	m_pClass = pClass;

	if(m_pClass->m_DataSize > 0)
		m_pData = (BYTE *)(m_pClass->m_pMem0->Alloc(m_pClass->m_DataSize));

	if(m_pClass->m_nRelLists)
	{
		m_RelList = (RVLARRAY *)(m_pClass->m_pMem0->Alloc(sizeof(RVLARRAY) * 
			m_pClass->m_nRelLists));

		memset(m_RelList, 0, sizeof(RVLARRAY) * m_pClass->m_nRelLists);
	}
}

CRVLObject2 * CRVLObject2::Create2(CRVLClass *pClass)
{
	CRVLObject2 *pObject = (CRVLObject2 *)(pClass->m_pMem0->Alloc(sizeof(CRVLObject2)));

	memcpy(pObject, this, sizeof(CRVLObject2));

	pObject->Create(pClass);

	pClass->Add(pObject);

	return pObject;
}

BOOL CRVLObject2::Match(CRVLObject2 *pObject, 
						int &MatchQuality)
{
	return FALSE;
}

int CRVLObject2::MaxMatchQuality()
{
	return 0;
}
/*
void CRVLObject2::GetMatchCandidates(CRVLCamera *pCamera,
									   CRVL2DCellArray *pCellArray,
									   double sigS,
									   CRVLMPtrChain *pMatchCandidateArray)
{

}
*/
int CRVLObject2::MatchCost(CRVLObject2 *pObject)
{
	return 0;
}


CRVLObject2 * CRVLObject2::GetMatch()
{
	return *((CRVLObject2 **)GetData(RVLOBJ2_DATA_MATCH_PTR));
}
/*
void CRVLObject2::GetRelList(CRVLMPtrChain2 *pRelList, 
							 int RelListID,
							 CRVLMem *pMem)
{
	pRelList->Create(m_RelList + m_pClass->m_iRelList[RelListID], pMem);
}*/

CRVLObject2 * CRVLObject2::Create3(CRVLClass *pClass)
{
	if(pClass->m_Flags & RVLCLS_FLAG_PERMANENT)
	{
		if(pClass->m_Flags & RVLCLS_FLAG_FULL)
			return NULL;

		CRVLObject2 **ppObject = (CRVLObject2 **)(pClass->m_pFirstFreeObjectPlace);

		CRVLObject2 *pObject = *ppObject;

		if(pObject)
		{
			pObject->Clear();

			pClass->Add(pObject);
		}
		else
		{
			pObject = Create2(pClass);

			*ppObject = pObject;
		}

		ppObject++;

		CRVLObject2 **pObjectArrayEnd = (CRVLObject2 **)(pClass->m_PermanentObjectArray) + 
			pClass->m_nPermanentObjectPlaces;

		// find next free object place

		CRVLObject2 *pNextObject;

		while(ppObject < pObjectArrayEnd)
		{
			pNextObject = *ppObject;

			if(pNextObject)
			{
				if(pNextObject->m_Flags & RVLOBJ2_FLAG_DELETED)
				{
					pClass->m_pFirstFreeObjectPlace = (void **)ppObject;

					return pObject;
				}
			}
			else
			{
				pClass->m_pFirstFreeObjectPlace = (void **)ppObject;

				return pObject;
			}

			ppObject++;
		}

		pClass->m_Flags |= RVLCLS_FLAG_FULL;

		return NULL;
	}
	else
		return Create2(pClass);
}

void CRVLObject2::Clear()
{
	m_Flags = 0x00000000;
	m_ParamFlags = 0x00000000;
	memset(m_RelList, 0, sizeof(RVLARRAY) * m_pClass->m_nRelLists);
}




#ifdef RVL_DEBUG

void CRVLObject2::DebugPrint(char *Name)
{
	fprintf(fpDebug, "%s%d\n", Name, m_Index);
}

#endif

void CRVLObject2::Save(	FILE *fp, 
						DWORD Flags)
{

}

void CRVLObject2::Load(	FILE *fp, 
						DWORD Flags)
{

}


///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 

void RVLAddObject(CRVLObject2 *pObject, CRVLClass *pClass)
{
	pObject->m_Index = (WORD)(pClass->m_nObjects);

	pClass->m_nObjects++;

	//pClass->m_ObjectList.Add(pObject);
}

void RVLDeselectAllObjects(CRVLClass *pClass)
{
	CRVLObject2 *pObject;

	//pClass->m_ObjectList.Start();

	/*while(pClass->m_ObjectList.m_pNext)
	{
		pObject = (CRVLObject2 *)(pClass->m_ObjectList.GetNext());

		pObject->m_Flags &= ~RVLOBJ2_FLAG_SELECTED;
	}*/
}

#ifdef RVL_DEBUG

void RVLDebugPrintObjects(CRVLMPtrChain *pObjectArray,
						  char *Name)
{
	CRVLObject2 *pObject;

	pObjectArray->Start();

	while(pObjectArray->m_pNext)
	{
		pObject = (CRVLObject2 *)(pObjectArray->GetNext());

		pObject->DebugPrint(Name);

		fprintf(fpDebug, "\n");
	}
}

#endif

void RestoreFromPermanentMem(CRVLClass *pClass)
{
	if(pClass->m_Flags & RVLCLS_FLAG_ARRAY)
		pClass->m_ObjectArray = 
			(void **)(pClass->m_pMem->Alloc(sizeof(void *) * pClass->m_nObjects));

	pClass->Clear();

	CRVLObject2 **ppObject = (CRVLObject2 **)(pClass->m_PermanentObjectArray);

	CRVLObject2 *pObject;

	if(pClass->m_Flags & RVLCLS_FLAG_FULL)
	{
	}
	else
	{
		while(*ppObject)
		{
			pObject = *ppObject;

			if((pObject->m_Flags & RVLOBJ2_FLAG_DELETED) == 0)
				pClass->Add(pObject);

			ppObject++;
		}
	}
}


void RVLClearFlags(CRVLClass *pClass)
{
	if(pClass->m_Flags & RVLCLS_FLAG_ARRAY)
	{
		void **pObjectArrayEnd = pClass->m_ObjectArray + pClass->m_nObjects;

		void **pvpObject;

		for(pvpObject = pClass->m_ObjectArray; pvpObject < pObjectArrayEnd; pvpObject++)
			((CRVLObject2 *)(*(pvpObject)))->m_Flags = 0x00000000;
	}
	else
	{
		//CRVLMPtrChain *pObjectList = &(pClass->m_ObjectList);

		/*pObjectList->Start();

		while(pObjectList->m_pNext)
			((CRVLObject2 *)(pObjectList->GetNext()))->m_Flags = 0x00000000;*/
	}
}
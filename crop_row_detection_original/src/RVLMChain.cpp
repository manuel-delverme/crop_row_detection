// RVLMChain.cpp: implementation of the CRVLMChain class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
#include "Platform.h"

#include <stdlib.h>
#include <string.h>
#include "RVLConst.h"
#include "RVLMem.h"
#include "RVLMChain.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLMChain::CRVLMChain()
{
	m_nElements = 0;
	m_pFirst = NULL;
}

CRVLMChain::CRVLMChain(CRVLMem *pMem, int DataSize)
{
	m_DataSize = DataSize;
	m_ElementSize = m_DataSize + sizeof(void *);
	m_pMem = pMem;
	m_nElements = 0;
	m_pFirst = NULL;
}

CRVLMChain::~CRVLMChain()
{
	
}

BYTE * CRVLMChain::Add(void *pData)
{
	BYTE *pElement = m_pMem->Alloc(m_ElementSize);
	*((BYTE **)pElement) = NULL;
	BYTE *pElementData = pElement + sizeof(BYTE *);
	memcpy(pElementData, pData, m_DataSize);
	if(m_pFirst == NULL)
		m_pFirst = m_pLast = pElement;
	else
	{
		*((BYTE **)m_pLast) = pElement;
		m_pLast = pElement;
	}
	m_nElements++;

	return pElementData;
}

void * CRVLMChain::GetNext()
{
	m_pCurrent = m_pNext;

	m_pNext = *((BYTE **)m_pCurrent);

	return m_pCurrent + sizeof(BYTE *);
}

void CRVLMChain::RemoveAll()
{
	m_nElements = 0;
	m_pFirst = NULL;
}

void CRVLMChain::Start()
{
	m_pNext = m_pFirst;
	m_pCurrent = NULL;
}

void CRVLMChain::InsertAt(void *pData, BYTE *pCurrent)
{
	BYTE *pElement = m_pMem->Alloc(m_ElementSize);
	memcpy(pElement + sizeof(BYTE *), pData, m_DataSize);

	if(pCurrent != NULL)
	{
		*((BYTE **)pElement) = *((BYTE **)pCurrent);
		*((BYTE **)pCurrent) = pElement;
	}
	else
	{
		*((BYTE **)pElement) = m_pFirst;
		m_pFirst = pElement;
	}

	m_nElements++;
}

void CRVLMChain::Create(CRVLMem *pMem, int DataSize)
{
	m_DataSize = DataSize;
	m_ElementSize = m_DataSize + sizeof(void *);
	m_pMem = pMem;
	m_nElements = 0;
	m_pFirst = NULL;
}


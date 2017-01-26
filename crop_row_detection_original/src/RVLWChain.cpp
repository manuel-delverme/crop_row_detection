// RVLWChain.cpp: Implementierung der Klasse CRVLWChain.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
#include "Platform.h"

#include <stdlib.h>
#include <string.h>
#include "RVLConst.h"
#include "RVLWChain.h"

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Konstruktion/Destruktion
//////////////////////////////////////////////////////////////////////

CRVLWChain::CRVLWChain()
{
	m_nElements = 0;
	m_pFirst = NULL;
}

CRVLWChain::~CRVLWChain()
{
	RemoveAll();
}

void CRVLWChain::Add(WORD Data)
{
	RVLWCHAIN_ELEMENT *pElement = new RVLWCHAIN_ELEMENT;
	pElement->Data = Data;
	pElement->pNext = NULL;
	if(m_pFirst == NULL)
		m_pFirst = m_pLast = pElement;
	else
	{
		m_pLast->pNext = pElement;
		m_pLast = pElement;
	}
	m_nElements++;
}

WORD CRVLWChain::GetNext()
{
	m_pCurrent = m_pNext;

	m_pNext = m_pCurrent->pNext;

	return m_pCurrent->Data;
}

void CRVLWChain::RemoveAll()
{
	m_pNext = m_pFirst;

	while(m_pNext != NULL)
	{
		m_pCurrent = m_pNext;

		m_pNext = m_pCurrent->pNext;

		delete m_pCurrent;
	}

	m_nElements = 0;
	m_pFirst = NULL;
}

void CRVLWChain::Start()
{
	m_pNext = m_pFirst;
}

void CRVLWChain::InsertAt(WORD Data, RVLWCHAIN_ELEMENT *pCurrent)
{
	RVLWCHAIN_ELEMENT *pElement = new RVLWCHAIN_ELEMENT;
	pElement->Data = Data;

	if(pCurrent != NULL)
	{
		pElement->pNext = pCurrent->pNext;
		pCurrent->pNext = pElement;
	}
	else
	{
		pElement->pNext = m_pFirst;
		m_pFirst = pElement;
	}

	m_nElements++;
}

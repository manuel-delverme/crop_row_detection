// RVLMem.cpp: implementation of the CRVLMem class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
#include "Platform.h"

#include <stdlib.h>
#include <string.h>
#include "RVLConst.h"
#include "RVLMem.h"

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLMem::CRVLMem()
{
	m_pFirstBlock = NULL;
}

CRVLMem::~CRVLMem()
{
	Free();
}

void CRVLMem::AddBlock()
{
	if(m_pFirstBlock != NULL)
		m_ppNextBlock = (BYTE **)m_pStartBlock;
				
	m_pStartBlock = (BYTE *)malloc(m_BlockSize);

	if(m_pFirstBlock != NULL)
		*m_ppNextBlock = m_pStartBlock;

	m_ppNextBlock = (BYTE **)m_pStartBlock;

	*m_ppNextBlock = NULL;

	m_pFreeMem = m_pStartBlock + sizeof(BYTE *);

	m_pEndBlock = m_pStartBlock + m_BlockSize;
}

BYTE *CRVLMem::AllocInNextBlock(int Size)
{
	m_ppNextBlock = (BYTE **)m_pStartBlock;

	if(*m_ppNextBlock == NULL)
		AddBlock();
	else
	{
		m_pStartBlock = *m_ppNextBlock;

		m_pFreeMem = m_pStartBlock + sizeof(BYTE *);

		m_pEndBlock = m_pStartBlock + m_BlockSize;
	}

	m_pAllocatedMem = m_pFreeMem;

	m_pFreeMem += Size;

	return m_pAllocatedMem;
}

void CRVLMem::Create(int BlockSize)
{
	m_BlockSize = BlockSize;

	m_pFirstBlock = NULL;

	AddBlock();

	m_pFirstBlock = m_pStartBlock;
}

void CRVLMem::Free()
{
	BYTE *pBlock = m_pFirstBlock;

	BYTE *pNextBlock;

	while(pBlock != NULL)
	{
		pNextBlock = *((BYTE **)(pBlock));

		free(pBlock);

		pBlock = pNextBlock;
	}	

	m_pFirstBlock = NULL;
}

void CRVLMem::Clear()
{
	m_pStartBlock = m_pFirstBlock;

	m_pFreeMem = m_pStartBlock + sizeof(BYTE *);

	m_pEndBlock = m_pStartBlock + m_BlockSize;
}

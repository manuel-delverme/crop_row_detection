#include "RVLCore.h"

CRVLQListArray::CRVLQListArray(void)
{
	m_ListArray = NULL;
}

CRVLQListArray::~CRVLQListArray(void)
{
	if(m_ListArray)
		delete[] m_ListArray;
}

void CRVLQListArray::Init(CRVLMem *pMem)
{
	if(pMem)
		m_ListArray = (RVLQLIST *)(pMem->Alloc(2 * m_Size * sizeof(RVLQLIST)));
	else
		m_ListArray = new RVLQLIST[2 * m_Size];

	m_EmptyListArray = m_ListArray + m_Size;

	InitListArray(m_EmptyListArray, m_ListArray);
}

void CRVLQListArray::Reset(void)
{
	memcpy(m_ListArray, m_EmptyListArray, m_Size * sizeof(RVLQLIST));
}

void CRVLQListArray::InitListArray(RVLQLIST *EmptyListArray,
								   RVLQLIST *ListArray)
{
	RVLQLIST *pEmptyListArrayEnd = EmptyListArray + m_Size;

	RVLQLIST *pList = ListArray;

	RVLQLIST *pEmptyList;

	for(pEmptyList = EmptyListArray; pEmptyList < pEmptyListArrayEnd; pEmptyList++, pList++)
		RVLQLIST_INIT2(pEmptyList, pList)
}


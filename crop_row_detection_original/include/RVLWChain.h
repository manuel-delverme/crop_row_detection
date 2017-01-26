// RVLWChain.h: Schnittstelle für die Klasse CRVLWChain.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLWCHAIN_H__E9193201_787A_41CF_B85F_BF0EEC679817__INCLUDED_)
#define AFX_RVLWCHAIN_H__E9193201_787A_41CF_B85F_BF0EEC679817__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

struct RVLWCHAIN_ELEMENT
{
	WORD Data;
	RVLWCHAIN_ELEMENT *pNext;
};


class CRVLWChain  
{
public:
	int m_nElements;
	RVLWCHAIN_ELEMENT *m_pFirst;
	RVLWCHAIN_ELEMENT *m_pLast;
	RVLWCHAIN_ELEMENT *m_pCurrent;
	RVLWCHAIN_ELEMENT *m_pNext;
	
public:
	void InsertAt(WORD Data, RVLWCHAIN_ELEMENT *pCurrent);
	void Start();
	void RemoveAll();
	WORD GetNext();
	void Add(WORD Data);
	CRVLWChain();
	virtual ~CRVLWChain();
};

#endif // !defined(AFX_RVLWCHAIN_H__E9193201_787A_41CF_B85F_BF0EEC679817__INCLUDED_)

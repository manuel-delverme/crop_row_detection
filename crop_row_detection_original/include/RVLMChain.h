// RVLMChain.h: interface for the CRVLMChain class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLMCHAIN_H__F28898AF_8BF8_4B20_AE88_85330779A2BF__INCLUDED_)
#define AFX_RVLMCHAIN_H__F28898AF_8BF8_4B20_AE88_85330779A2BF__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CRVLMChain  
{
public:
	int m_nElements;
	int m_DataSize;
	BYTE *m_pFirst;
	BYTE *m_pLast;
	BYTE *m_pCurrent;
	BYTE *m_pNext;
	CRVLMem *m_pMem;
private:
	int m_ElementSize;
	
public:
	inline void * GetLast()
	{
		return m_pLast + sizeof(BYTE *);
	};
	void Create(CRVLMem *pMem, int DataSize);
	void InsertAt(void *pData, BYTE *pCurrent);
	void Start();
	void RemoveAll();
	void * GetNext();
	BYTE * Add(void *pData);
	CRVLMChain(CRVLMem *pMem, int DataSize);
	CRVLMChain();
	virtual ~CRVLMChain();
};

#endif // !defined(AFX_RVLMCHAIN_H__F28898AF_8BF8_4B20_AE88_85330779A2BF__INCLUDED_)

// RVLBuffer.h: interface for the CRVLBuffer class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLBUFFER_H__402B0E2F_FCFD_4D06_8F0E_E2E286BF34AD__INCLUDED_)
#define AFX_RVLBUFFER_H__402B0E2F_FCFD_4D06_8F0E_E2E286BF34AD__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CRVLBuffer  
{
public:
	BOOL m_bOwnData;
	void **DataBuff;
	void **ppData, **pEnd;

public:
	CRVLBuffer();
	virtual ~CRVLBuffer();

};

#endif // !defined(AFX_RVLBUFFER_H__402B0E2F_FCFD_4D06_8F0E_E2E286BF34AD__INCLUDED_)

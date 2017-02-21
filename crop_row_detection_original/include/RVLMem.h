// RVLMem.h: interface for the CRVLMem class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLMEM_H__2BB05049_AE8C_4F63_9DF3_B85744566F5D__INCLUDED_)
#define AFX_RVLMEM_H__2BB05049_AE8C_4F63_9DF3_B85744566F5D__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define RVLMEM_ALLOC(pMem, Size, pAllocatedMem)	{pAllocatedMem = pMem->m_pFreeMem; pMem->m_pFreeMem += Size; if(pMem->m_pFreeMem >= pMem->m_pEndBlock) pAllocatedMem = pMem->AllocInNextBlock(Size);}
#define RVLMEM_ALLOC_STRUCT(pMem, Type, pAllocatedMem)	{pAllocatedMem = (Type *)(pMem->m_pFreeMem); pMem->m_pFreeMem += sizeof(Type); if(pMem->m_pFreeMem >= pMem->m_pEndBlock) pAllocatedMem = (Type *)(pMem->AllocInNextBlock(sizeof(Type)));}
#define RVLMEM_ALLOC_STRUCT_ARRAY(pMem, Type, n, pAllocatedMem)	{pAllocatedMem = (Type *)(pMem->m_pFreeMem); pMem->m_pFreeMem += (n * sizeof(Type)); if(pMem->m_pFreeMem >= pMem->m_pEndBlock) pAllocatedMem = (Type *)(pMem->AllocInNextBlock(sizeof(Type)));}

class CRVLMem  
{
public:
	BYTE *m_pFreeMem;
	BYTE *m_pEndBlock;
	BYTE *m_pStartBlock;	
	
private:
	BYTE *m_pFirstBlock;
	int m_BlockSize;
	BYTE *m_pAllocatedMem;
	BYTE **m_ppNextBlock;

public:
	void Clear();
	void Free();
	void Create(int BlockSize);
	void AddBlock();
	BYTE *AllocInNextBlock(int Size);
	CRVLMem();
	virtual ~CRVLMem();
	inline BYTE *Alloc(int Size)
	{
		m_pAllocatedMem = m_pFreeMem;

		m_pFreeMem += Size;

		if(m_pFreeMem >= m_pEndBlock)
			return AllocInNextBlock(Size);
		else
			return m_pAllocatedMem;
	};

	inline BYTE *CAlloc(int Size)
	{
		Alloc(Size);

		memset(m_pAllocatedMem, 0x00, Size);

		return m_pAllocatedMem;
	};
};

#endif // !defined(AFX_RVLMEM_H__2BB05049_AE8C_4F63_9DF3_B85744566F5D__INCLUDED_)

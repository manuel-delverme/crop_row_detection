// RVLEDT.h: interface for the CRVLEDT class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLEDT_H__093E4959_5B7D_4E20_AA26_110314642404__INCLUDED_)
#define AFX_RVLEDT_H__093E4959_5B7D_4E20_AA26_110314642404__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#define RVLEDT_FLAG_EDGE_CONTOURS		0x00000001
#define RVLEDT_STOP_WAVE				0xfffffffe

struct RVLEDT_PIX
{
	unsigned int d2;
	short int dx, dz;
};

struct RVLEDT_PIX_ARRAY
{
	int Width, Height;
	RVLEDT_PIX *pPix;
};

struct RVLEDT_BUCKET_ENTRY
{
	RVLEDT_PIX *pPix;
	short int dx, dz;
};

void RVLEDTDisplay(RVLEDT_PIX_ARRAY *pEDTImage,
				   PIX_ARRAY *pDisplayImage);

class CRVLEDT 
{
public:
	DWORD m_Flags;
	unsigned int m_maxd2;
	int m_dpNeighbor[4];
	RVLEDT_BUCKET_ENTRY *m_BucketPtrArray[4];
	//RVLRECT *m_pROI;
private:
	unsigned int m_d2InitWave;
	CRVLBuffer *m_pOutBuff;
	//CRVLMChain **m_BucketPtrArray;
	//CRVLMem m_Mem;
	//CRVLMChain **m_ppBucket, **m_pLastBucketPtr;

public:
	/*void Init(	RVLEDT_PIX_ARRAY *pEDTImage, 
				CRVLMem *pMem, 
				RVLRECT *pROI = NULL);
	void Border(RVLEDT_PIX_ARRAY *pEDTImage,
				RVLRECT *pROI = NULL);*/
	void InitEDTPixArray(RVLEDT_PIX_ARRAY *pEDTImage);
	//void Add2Bucket(RVLEDT_BUCKET_ENTRY *pBucketEntry);
	/*void Apply(CRVLMPtrChain *pBorderIn, CRVLMPtrChain *pBorderOut,
		RVLEDT_PIX_ARRAY *pEDTImage, CRVLBuffer *pOutBuff = NULL);*/
	CRVLEDT();
	virtual ~CRVLEDT();
};

#endif // !defined(AFX_RVLEDT_H__093E4959_5B7D_4E20_AA26_110314642404__INCLUDED_)

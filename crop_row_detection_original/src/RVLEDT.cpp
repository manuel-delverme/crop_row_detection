// RVLEDT.cpp: implementation of the CRVLEDT class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "RVLCore.h"
#include "RVLEDT.h"

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLEDT::CRVLEDT()
{
	m_Flags = 0x00000000;
	m_maxd2 = 0xffffffff;
	m_pOutBuff = NULL;
	//m_pROI = NULL;
}

CRVLEDT::~CRVLEDT()
{

}
/*
void CRVLEDT::Apply(CRVLMPtrChain *pBorderIn, CRVLMPtrChain *pBorderOut, 
					RVLEDT_PIX_ARRAY *pEDTImage, CRVLBuffer *pOutBuff)
{
	if(pOutBuff)
		m_pOutBuff = pOutBuff;

	// m_d2InitWave = (m_Flags & RVLEDT_FLAG_EDGE_CONTOURS ? 0xffffffff : 2);
	m_d2InitWave = 0xffffffff;

	int w = pEDTImage->Width;
	int h = pEDTImage->Height;

	m_dpNeighbor[0] = -1;
	m_dpNeighbor[1] = 1;
	m_dpNeighbor[2] = -w;
	m_dpNeighbor[3] = w;

	int *pEnddpNeighbor = m_dpNeighbor + 4;

	int dxNeighbor[4] = {-1, 1, 0, 0};
	int dzNeighbor[4] = {0, 0, -1, 1};

	// initialize buckets
	
	int BucketSize = w * h;

	WORD iBucket;
	RVLEDT_BUCKET_ENTRY *EndBucketPtrArray[4];

	for(iBucket = 0; iBucket < 4; iBucket++)
		EndBucketPtrArray[iBucket] = m_BucketPtrArray[iBucket];*/

	/*
	int Diagonal = (int)floor(sqrt((double)(pEDTImage->Width * pEDTImage->Width + 
		pEDTImage->Height * pEDTImage->Height))) + 1;

	m_BucketPtrArray = new CRVLMChain *[Diagonal];

	memset(m_BucketPtrArray, 0, Diagonal * sizeof(CRVLMChain *));

	m_pLastBucketPtr = m_BucketPtrArray;

	m_Mem.Create(pEDTImage->Width * pEDTImage->Height * (sizeof(RVLEDT_BUCKET_ENTRY) + 
		sizeof(void *)));
	*/

	// get border

	/*m_pOutBuff->ppData = m_pOutBuff->DataBuff;

	RVLEDT_BUCKET_ENTRY *pBucketEntry = m_BucketPtrArray[0];

	RVLEDT_PIX *pEDTPix;

	pBorderIn->Start();

	while(pBorderIn->m_pNext)
	{
		pEDTPix = (RVLEDT_PIX *)(pBorderIn->GetNext());

		pEDTPix->d2 = 0;
		pEDTPix->dx = pEDTPix->dz = 0;

		pBucketEntry->pPix = pEDTPix;
		pBucketEntry->dx = pBucketEntry->dz = 0;
		pBucketEntry++;	
		
		//*(m_pOutBuff->ppData++) = pEDTPix;
	}

	EndBucketPtrArray[0] = pBucketEntry;

	if((m_Flags & RVLEDT_FLAG_EDGE_CONTOURS) == 0)
	{
		pBorderOut->Start();

		while(pBorderOut->m_pNext)
		{
			pEDTPix = (RVLEDT_PIX *)(pBorderOut->GetNext());

			pEDTPix->d2 = 2;
		}
	}

	RVLEDT_PIX *pEDTNeighborPix;
	int *pdpNeighbor;*/
	
	/*
	RVLEDT_PIX *pEDTPix = pEDTImage->pPix + pEDTImage->Width;
	RVLEDT_BUCKET_ENTRY *pBucketEntry = m_BucketPtrArray[0];
	
	int u, v;
	int *pdpNeighbor;
	RVLEDT_PIX *pEDTNeighborPix;
	//RVLEDT_BUCKET_ENTRY BucketEntry;

	for(v = 1; v < pEDTImage->Height - 1; v++)
	{
		pEDTPix++;

		for(u = 1; u < pEDTImage->Width - 1; u++, pEDTPix++)
		{
			if(pEDTPix->d2 != 0)
				continue;

			for(pdpNeighbor = m_dpNeighbor; pdpNeighbor < pEnddpNeighbor; pdpNeighbor++)
			{
				pEDTNeighborPix = pEDTPix + (*pdpNeighbor);

				if(pEDTNeighborPix->d2 != 0)
					break;
			}

			if(pEDTNeighborPix->d2 != 0)
			{
				pBucketEntry->pPix = pEDTPix;
				pBucketEntry->dx = pBucketEntry->dz = 0;
				pBucketEntry++;

				//BucketEntry.pPix = pEDTPix;
				//BucketEntry.dx = BucketEntry.dz = 0;

				//Add2Bucket(&BucketEntry);
			}	
		}

		pEDTPix++;
	}

	EndBucketPtrArray[0] = pBucketEntry;
	*/

	// region growing	

	/*unsigned int d = 0;
	int *sqrt_LookUpTable = RVLGetsqrt_LookUpTable();
	//int counter = 0;

	RVLEDT_BUCKET_ENTRY *pBucketEntry2;
	unsigned int d2, dNeighbor;
	int dx, dz;
	int *pdxNeighbor, *pdzNeighbor;
	//CRVLMChain **ppBucket;
	WORD i, iBucket2;
	int iPix;
	RVLEDT_BUCKET_ENTRY *BucketPtrArray;
	RVLEDT_BUCKET_ENTRY **ppEndBucket, **ppEndBucket2;
	int nsqrt, xsqrt;
	BOOL bStop;

	//for(ppBucket = m_BucketPtrArray; ppBucket <= m_pLastBucketPtr; ppBucket++)
	while(TRUE)
	{
		iBucket = (WORD)(d & 0x00000003);

		BucketPtrArray = m_BucketPtrArray[iBucket];
		ppEndBucket = EndBucketPtrArray + iBucket;

		//if(*ppBucket == NULL)
		//	continue;

		//(*ppBucket)->Start();

		//while((*ppBucket)->m_pNext)
		for(pBucketEntry = BucketPtrArray; pBucketEntry < *ppEndBucket; pBucketEntry++)
		{
			//pBucketEntry = (RVLEDT_BUCKET_ENTRY *)((*ppBucket)->GetNext());
			pdxNeighbor = dxNeighbor;
			pdzNeighbor = dzNeighbor;

			pEDTPix = pBucketEntry->pPix;

			iPix = (int)(pEDTPix - pEDTImage->pPix);

			pdpNeighbor = m_dpNeighbor;

			for(i = 0; i < 4; i++, pdpNeighbor++, pdxNeighbor++, pdzNeighbor++)
			{
				switch(i){
				case 0:
					if(iPix % w == 0)
						continue;

					break;
				case 1:
					if(iPix % w == w - 1)
						continue;

					break;
				case 2:
					if(iPix < w)
						continue;

					break;
				case 3:
					if(iPix >= (h - 1) * w)
						continue;
				}

				pEDTNeighborPix = pEDTPix + (*pdpNeighbor);

				bStop = (pEDTNeighborPix->d2 == RVLEDT_STOP_WAVE);

				if(pBucketEntry->pPix->d2 == 0)
					if(pEDTNeighborPix->d2 != m_d2InitWave && !bStop)
						continue;

				dx = pBucketEntry->dx + *pdxNeighbor;
				dz = pBucketEntry->dz + *pdzNeighbor;
				d2 = dx * dx + dz * dz;

				if(d2 < pEDTNeighborPix->d2 && d2 <= m_maxd2)
				{
					pEDTNeighborPix->d2 = d2;
					pEDTNeighborPix->dx = dx;
					pEDTNeighborPix->dz = dz;

					if(!bStop)
					{
						// dNeighbor = sqrt(d2)

						if(d2 > 65280)
						{
							nsqrt = 0;
							xsqrt = d2;

							while(xsqrt > 65280)
							{
								xsqrt = (xsqrt >> 2);
								nsqrt++;
							}

							dNeighbor = ((sqrt_LookUpTable[xsqrt] + 1) << nsqrt) - 1;
						}
						else
							dNeighbor = sqrt_LookUpTable[d2];

						/////

						iBucket2 = (WORD)(dNeighbor & 0x00000003);
						ppEndBucket2 = EndBucketPtrArray + iBucket2;
						pBucketEntry2 = *ppEndBucket2;
						pBucketEntry2->pPix = pEDTNeighborPix;
						pBucketEntry2->dx = dx;
						pBucketEntry2->dz = dz;
						(*ppEndBucket2)++;
					}

					*(m_pOutBuff->ppData++) = pEDTNeighborPix;
					//counter++;
					/*
					BucketEntry.pPix = pEDTNeighborPix;
					BucketEntry.dx = dx;
					BucketEntry.dz = dz;
					Add2Bucket(&BucketEntry);				
					*/
				/*}
			}	// for each neighboring pixel	
		}	// for each entry corresponding to distance d

		if(EndBucketPtrArray[0] == m_BucketPtrArray[0] &&
			EndBucketPtrArray[1] == m_BucketPtrArray[1] &&
			EndBucketPtrArray[2] == m_BucketPtrArray[2] &&
			EndBucketPtrArray[3] == m_BucketPtrArray[3])
			break;

		*ppEndBucket = m_BucketPtrArray[iBucket];

		d++;
	}	// for each distance d

	m_pOutBuff->pEnd = m_pOutBuff->ppData;

	// deallocate memory

	//m_Mem.Free();

	//delete[] m_BucketPtrArray;
}	*/// Apply

/*
inline void CRVLEDT::Add2Bucket(RVLEDT_BUCKET_ENTRY *pBucketEntry)
{
	m_ppBucket = m_BucketPtrArray + rtsqrt(pBucketEntry->pPix->d2);

	if(*m_ppBucket == NULL)
	{
		*m_ppBucket = new CRVLMChain(&m_Mem, sizeof(RVLEDT_BUCKET_ENTRY));

		if(m_ppBucket > m_pLastBucketPtr)
			m_pLastBucketPtr = m_ppBucket;
	}

	(*m_ppBucket)->Add((void *)pBucketEntry);
}
*/

void CRVLEDT::InitEDTPixArray(RVLEDT_PIX_ARRAY *pEDTImage)
{
	memset(pEDTImage->pPix, 0x00, (pEDTImage->Width + 1) * sizeof(RVLEDT_PIX));

	memset(pEDTImage->pPix + pEDTImage->Width + 1, 0xff, (pEDTImage->Width * 
		(pEDTImage->Height - 2) - 2) * sizeof(RVLEDT_PIX));

	memset(pEDTImage->pPix + pEDTImage->Width * (pEDTImage->Height - 1) - 1, 0x00,
		(pEDTImage->Width + 1) * sizeof(RVLEDT_PIX));

	RVLEDT_PIX *pEDTPix = pEDTImage->pPix + 2 * pEDTImage->Width - 1;

	int v;

	for(v = 1; v < pEDTImage->Height - 2; v++, pEDTPix += pEDTImage->Width)
		memset(pEDTPix, 0x00, 2 * sizeof(RVLEDT_PIX));
}
/*
void CRVLEDT::Border(RVLEDT_PIX_ARRAY *pEDTImage,
					 RVLRECT *pROI)
{
	int Width = pEDTImage->Width;
	int Height = pEDTImage->Height;

	RVLRECT ROI;

	if(pROI)
		ROI = *pROI;
	else
	{
		ROI.top = 1;
		ROI.bottom = pEDTImage->Height - 2;
		ROI.left = 1;
		ROI.right = pEDTImage->Width - 2;
	}

	memset(pEDTImage->pPix, 0x00, ROI.top * Width * sizeof(RVLEDT_PIX));	

	memset(pEDTImage->pPix + (ROI.bottom + 1) * Width, 0x00, 
		(Height - ROI.bottom - 1) * Width * sizeof(RVLEDT_PIX));

	RVLEDT_PIX *pEDTPix = pEDTImage->pPix + ROI.top * Width;

	int LeftBorderWidth = ROI.left;
	int LeftBorderSize = LeftBorderWidth * sizeof(RVLEDT_PIX);
	int RightBorderWidth = (Width - ROI.right - 1);
	int RightBorderSize = RightBorderWidth * sizeof(RVLEDT_PIX);
	int ROIWidth = (ROI.right - ROI.left + 1);
	int ROISize = ROIWidth * sizeof(RVLEDT_PIX);
	int RightBorderOffset = LeftBorderWidth + ROIWidth;

	int v;
	int bottom = ROI.bottom;

	for(v = ROI.top; v <= bottom; v++, pEDTPix += Width)
	{
		memset(pEDTPix, 0x00, LeftBorderSize);
		memset(pEDTPix + LeftBorderWidth, 0xff, ROISize);
		memset(pEDTPix + RightBorderOffset, 0x00, RightBorderSize);
	}		
}*/


void RVLEDTDisplay(RVLEDT_PIX_ARRAY *pEDTImage,
				   PIX_ARRAY *pDisplayImage)
{
	RVLEDT_PIX *pEndEDTPix = pEDTImage->pPix + 
		pEDTImage->Height * pEDTImage->Width;

	RVLEDT_PIX *pEDTPix;
	unsigned char *pPix = pDisplayImage->pPix;
	double I;

	for(pEDTPix = pEDTImage->pPix; pEDTPix < pEndEDTPix; 
		pEDTPix++, pPix++)
	{
		if(pEDTPix->d2 == 0xff)
			*pPix = 0xff;
		else
		{
			I = DOUBLE2INT(sqrt((double)(pEDTPix->d2)) * 4.0);

			if(I > 255)
				I = 255;

			*pPix = (unsigned char)I;
		}
	}
}

/*
void CRVLEDT::Init(RVLEDT_PIX_ARRAY *pEDTImage, 
				   CRVLMem *pMem, 
				   RVLRECT *pROI)
{
	int ImageSize = pEDTImage->Width * pEDTImage->Height;

	WORD iBucket;

	for(iBucket = 0; iBucket < 4; iBucket++)
		m_BucketPtrArray[iBucket] = (RVLEDT_BUCKET_ENTRY *)
			(pMem->Alloc(ImageSize * sizeof(RVLEDT_BUCKET_ENTRY)));

	Border(pEDTImage, pROI);

	m_pOutBuff->DataBuff = (void **)(pMem->Alloc(ImageSize * sizeof(void *)));
	m_pOutBuff->m_bOwnData = FALSE;

	int maxr = (pEDTImage->Width > pEDTImage->Height ? 
		pEDTImage->Height : pEDTImage->Width);

	m_maxd2 = maxr * maxr;
}
*/


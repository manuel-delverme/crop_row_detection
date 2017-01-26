// RVLPCIGR.cpp: implementation of the CRVLPCIGR class.
//
//////////////////////////////////////////////////////////////////////

#include "RVLCore.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLAImage::CRVLAImage()
{
	m_bOwnData = FALSE;
	//m_pPix = NULL;
	//m_ElementSize = sizeof(RVLAPIX);
	//m_EDTImage.pPix = NULL;
	//m_EDTDisparityImage.pPix = NULL;
	m_iNeighborMem = NULL;
	//m_2DRegionMap = NULL;
	//m_2DRegionMap2 = NULL;
	m_DistanceLookUpTable = NULL;
	//m_pROI = NULL;
}

CRVLAImage::~CRVLAImage()
{
	/*if(m_bOwnData)
		if(m_pPix)
			delete[] m_pPix;*/

	/*if(m_EDTImage.pPix != NULL)
		delete[] m_EDTImage.pPix;	

	if(m_EDTDisparityImage.pPix != NULL)
		delete[] m_EDTDisparityImage.pPix;*/

	if(m_iNeighborMem)
		delete[] m_iNeighborMem;

	/*f(m_2DRegionMap)
		delete[] m_2DRegionMap;

	if(m_2DRegionMap2)
		delete[] m_2DRegionMap2;*/

	if(m_DistanceLookUpTable)
		delete[] m_DistanceLookUpTable;
}

void CRVLAImage::Create(unsigned char *PixArray)
{
//	Clear();

	unsigned char *pPix = PixArray;
	//RVLAPIX *pAPix = m_pPix;

	int u, v;

}



void CRVLAImage::Init()
{
	m_Width = m_pCameraL->Width;
	m_Height = m_pCameraL->Height;

	m_dpNeighbor4[0] = 1;
	m_dpNeighbor4[1] = m_Width;
	m_dpNeighbor4[2] = -1;
	m_dpNeighbor4[3] = -m_Width;

	m_dpNeighbor8[0] = 1;
	m_dpNeighbor8[1] = m_Width + 1;
	m_dpNeighbor8[2] = m_Width;
	m_dpNeighbor8[3] = m_Width - 1;
	m_dpNeighbor8[4] = -1;
	m_dpNeighbor8[5] = -m_Width - 1;
	m_dpNeighbor8[6] = -m_Width;
	m_dpNeighbor8[7] = -m_Width + 1;

	m_idIDirection[0][0] = 5;
	m_idIDirection[0][1] = 6;
	m_idIDirection[0][2] = 7;
	m_idIDirection[1][0] = 4;
	m_idIDirection[1][2] = 0;
	m_idIDirection[2][0] = 3;
	m_idIDirection[2][1] = 2;
	m_idIDirection[2][2] = 1;

	m_iNeighborMem = new char[4 * (m_Width + 1) + 1];
	m_iNeighbor = m_iNeighborMem + 2 * (m_Width + 1);

	memset(m_iNeighborMem, 0xff, (4 * (m_Width + 1) + 1) * sizeof(char));

	m_iNeighbor[1]				= 0;
	m_iNeighbor[m_Width + 1]	= 1;
	m_iNeighbor[m_Width]		= 2;
	m_iNeighbor[m_Width - 1]	= 3;
	m_iNeighbor[-1]				= 4;
	m_iNeighbor[-m_Width - 1]	= 5;
	m_iNeighbor[-m_Width]		= 6;
	m_iNeighbor[-m_Width + 1]	= 7;

	m_NeighborLimit[0] = m_Width - 1;
	m_NeighborLimit[1] = m_Height - 1;
	m_NeighborLimit[2] = 0;
	m_NeighborLimit[3] = 0;

	int i, j, k;

	int du0 = -1;
	int dv0 = -1;

	int du, dv, iTmp;

	for(i = 0; i < 4; i++)
	{
		k = ((i + 3) & 3);

		du = dv0;
		dv = -du0;

		for(j = 0; j < 3; j++)
		{
			m_dpContourFollow[i][k] = 2 * ((du - dv0) + (dv + du0) * m_Width) + k - i;

			k = ((k + 1) & 3);

			iTmp = du;
			du = -dv;
			dv = iTmp;
		}

		iTmp = du0;
		du0 = -dv0;
		dv0 = iTmp;
	}

	int ImageSize = m_Width * m_Height;

	m_DistanceLookUpTable = new int[ImageSize];

	int *pDist = m_DistanceLookUpTable;

	int u, v;

	for(v = 0; v < m_Height; v++)
		for(u = 0; u < m_Width; u++, pDist++)
			*pDist = DOUBLE2INT(1000.0 * sqrt((double)(u * u + v * v)));

	///// initialize RVL classes

	// m_C2DRegion

	/*m_C2DRegion.m_pMem0 = m_pMem;
	m_C2DRegion.m_pMem = m_pMem;
	m_C2DRegion.m_pMem2 = m_pMem2;

	//m_C2DRegion.m_pRelList = &m_RelList;

	m_C2DRegion.m_iRelListNeighbors = m_C2DRegion.m_nRelLists++;

	m_C2DRegion.m_iDataParentPtr = m_C2DRegion.m_DataSize;
	m_C2DRegion.m_DataSize += sizeof(CRVLObject2 *);

	m_C2DRegion.m_iDataGrandParentPtr = m_C2DRegion.m_DataSize;
	m_C2DRegion.m_DataSize += sizeof(CRVLObject2 *);*/

	//m_C2DRegion.AppendData(RVLOBJ2_DATA_NODE_PTR);

//	RVLCreateC2D(&m_C2DRegion);

	/*m_2DRegionMap = new CRVL2DRegion2 *[ImageSize];

	m_2DRegionMap2 = new RVLAPIX_2DREGION_PTR[ImageSize];*/

	// m_C2DRegion2

	/*m_C2DRegion2.m_pMem0 = m_pMem;
	m_C2DRegion2.m_pMem = m_pMem;
	m_C2DRegion2.m_pMem2 = m_pMem2;*/

	//m_C2DRegion2.m_pRelList = &m_RelList;

	/*m_C2DRegion2.m_iDataParentPtr = m_C2DRegion2.m_DataSize;
	m_C2DRegion2.m_DataSize += sizeof(CRVLObject2 *);

	m_C2DRegion2.m_iRelListComponents = m_C2DRegion2.m_nRelLists++;
	m_C2DRegion2.m_iRelListContours = m_C2DRegion2.m_nRelLists++;*/

//	RVLCreateC2D(&m_C2DRegion2);

	// m_C2DRegion3

	/*m_C2DRegion3.m_pMem0 = m_pMem;
	m_C2DRegion3.m_pMem = m_pMem;
	m_C2DRegion3.m_pMem2 = m_pMem2;

	//m_C2DRegion3.m_pRelList = &m_RelList;

	m_C2DRegion3.m_iRelListComponents = m_C2DRegion3.m_nRelLists++;
	m_C2DRegion3.m_iRelListElements = m_C2DRegion3.m_nRelLists++;*/

//	RVLCreateC2D(&m_C2DRegion3);

	// initialize Relation list

	//m_RelList.m_pMem = m_pMem;

	// initialize EDT

	//m_EDT.m_Flags = RVLEDT_FLAG_EDGE_CONTOURS;

	int maxr = (m_Width > m_Height ? 
		m_Height : m_Width);

	//m_EDT.m_maxd2 = maxr * maxr;

	//m_EDT.m_maxd2 = 256;

	/*m_EDTImage.Width = m_Width;
	m_EDTImage.Height = m_Height;*/

	/*if(m_EDTImage.pPix != NULL)
		delete[] m_EDTImage.pPix;

	m_EDTImage.pPix = (RVLEDT_PIX *)malloc(ImageSize * sizeof(RVLEDT_PIX));


	// initialize EDT (Disparity Image)

	m_EDTDisparityImage.Width = m_Width;
	m_EDTDisparityImage.Height = m_Height;

	if(m_EDTDisparityImage.pPix != NULL)
		delete[] m_EDTDisparityImage.pPix;*/

	//m_EDTDisparityImage.pPix = (RVLEDT_PIX *)malloc(ImageSize * sizeof(RVLEDT_PIX));
}




class CRVLAImage
{
public:
	CRVLMem *m_pMem, *m_pMem2;
	int m_Width, m_Height;
	int m_ElementSize;
	//RVLAPIX *m_pPix;
	BOOL m_bOwnData;
	int m_dpNeighbor4[4];
	int m_dpNeighbor8[8];
	int m_NeighborLimit[4];
	int m_dpContourFollow[4][4];
	BYTE m_idIDirection[3][3];
	CRVLCamera *m_pCameraL, *m_pCameraR;
	char *m_iNeighbor;
	int *m_DistanceLookUpTable;

private:
	char *m_iNeighborMem;

public:
	void Init();
	void Clear();
	void Create(unsigned char *PixArray);
	void CreateBorder();
	CRVLAImage();
	virtual ~CRVLAImage();
};




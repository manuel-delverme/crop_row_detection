struct RVLARRAY
{
	BYTE *pFirst;
	BYTE *pEnd;
};

struct RVLARRAY2
{
	BYTE *pFirst;
	BYTE *pLast;
	BYTE *pData;
};

struct RVL2DARRAY
{
	BYTE *m_vp;
	int m_ElementSize;
	int m_Width;
	int m_Height;
};

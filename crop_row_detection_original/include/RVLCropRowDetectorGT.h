#pragma once

#define GTImageWidth	320
#define	GTImageHeight	240

struct RVLCRD_GT_CURVE_SEGMENT
{
	double u;
	int v;
	double du;
	double a0, a1, a2, a3;
};

struct RVLCRD_GT_CURVE
{
	int n;
	RVLCRD_GT_CURVE_SEGMENT *Segment;
};

void RVLCRDGTMouseCallback(int event, int x, int y, int flags, void* pData);

class CRVLCropRowDetectorGT
{
public:
	CRVLCropRowDetectorGT(void);
	virtual ~CRVLCropRowDetectorGT(void);
	void Init();
	void Display();
	void Clear();
	void RemoveSegment();
	void Edit(void);
	void AddSegment(int u, int v);
	void ComputeCurve(int iCurve);
	void ComputeCRParams(void);
	void SelectSegment(int u, int v);
	void Save(void);
	bool Load(void);
	FILE * OpenFile(char *mode, char *Extension = ".gtr");
	void SaveCRParams(void);
	void EvaluateParams(int *c, int *d, double sigma);
	void EvaluateParams(double *u, double sigma);
	void SaveImage(int experimentNum);
	void SaveUCoordinates(int *c, int*d);

public:
	IplImage *m_pInputImage;
	IplImage *m_pDisplay;
	int m_iCurve;
	RVLCRD_GT_CURVE m_Curve[2];
	double *m_c;
	double *m_d;
	int m_v0;
	RVLCRD_GT_CURVE_SEGMENT *m_pSelectedSegment;
	char *m_InputImageFileName;
	double m_E;
	float m_widthRatio;
	float m_heightRatio;
};

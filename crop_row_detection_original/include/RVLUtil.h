#define APPROX_ZERO		1e-10
#define DOUBLE2INT(x)	(int)floor(x + 0.5)
#define DOUBLE2BYTE(x)	(BYTE)floor(x + 0.5)
// Tgt = Src(3x1)
#define RVLCOPY3VECTOR(Src, Tgt)	Tgt[0] = Src[0]; Tgt[1] = Src[1]; Tgt[2] = Src[2];
// Tgt = Src(3x3)
#define RVLCOPYMX3X3(Src, Tgt)	Tgt[0] = Src[0]; Tgt[1] = Src[1]; Tgt[2] = Src[2]; Tgt[3] = Src[3]; Tgt[4] = Src[4]; Tgt[5] = Src[5]; Tgt[6] = Src[6]; Tgt[7] = Src[7]; Tgt[8] = Src[8];
// Tgt = Src(3x3)'
#define RVLCOPYMX3X3T(Src, Tgt)	Tgt[0] = Src[0]; Tgt[1] = Src[3]; Tgt[2] = Src[6]; Tgt[3] = Src[1]; Tgt[4] = Src[4]; Tgt[5] = Src[7]; Tgt[6] = Src[2]; Tgt[7] = Src[5]; Tgt[8] = Src[8];
// y = i-th column of X(3x3)
#define RVLCOPYCOLMX3X3(X, i, y)	y[0] = X[i]; y[1] = X[3+i]; y[2] = X[6+i];
// i-th column of Y(3x3) = x
#define RVLCOPYTOCOL3(x, i, Y)	Y[i] = x[0]; Y[3+i] = x[1]; Y[6+i] = x[2];
// Z = X(3x3) + Y(3x3)
#define RVLSUMMX3X3(X, Y, Z)	Z[0] = X[0] + Y[0]; Z[1] = X[1] + Y[1]; Z[2] = X[2] + Y[2]; Z[3] = X[3] + Y[3]; Z[4] = X[4] + Y[4]; Z[5] = X[5] + Y[5]; Z[6] = X[6] + Y[6]; Z[7] = X[7] + Y[7]; Z[8] = X[8] + Y[8]; 
// Z = X(3x3) - Y(3x3)
#define RVLDIFMX3X3(X, Y, Z)	Z[0] = X[0] - Y[0]; Z[1] = X[1] - Y[1]; Z[2] = X[2] - Y[2]; Z[3] = X[3] - Y[3]; Z[4] = X[4] - Y[4]; Z[5] = X[5] - Y[5]; Z[6] = X[6] - Y[6]; Z[7] = X[7] - Y[7]; Z[8] = X[8] - Y[8]; 
// Z = X(3x3) + X(3x3)' (only diagonal + upper triangle are computed)
#define RVLSUMMX3X3T2UT(X, Y, Z)	Z[0] = X[0] + X[0]; Z[1] = X[1] + X[3]; Z[2] = X[2] + Y[6]; Z[4] = X[4] + Y[4]; Z[5] = X[5] + Y[7]; Z[8] = X[8] + Y[8]; 
// Z = X(3x3) + Y(3x3) (only diagonal + upper triangle are computed)
#define RVLSUMMX3X3UT(X, Y, Z)	Z[0] = X[0] + Y[0]; Z[1] = X[1] + Y[1]; Z[2] = X[2] + Y[2]; Z[4] = X[4] + Y[4]; Z[5] = X[5] + Y[5]; Z[8] = X[8] + Y[8]; 
// X = 0(3x1)
#define RVLNULL3VECTOR(X)	X[0] = X[1] = X[2] = 0.0;
// X = 0(3x3)
#define RVLNULLMX3X3(X)	X[0] = X[1] = X[2] = X[3] = X[4] = X[5] = X[6] = X[7] = X[8] = 0.0;
// X = I(3x3)
#define RVLUNITMX3(X)	X[0] = X[4] = X[8] = 1.0; X[1] = X[2] = X[3] = X[5] = X[6] = X[7] = 0.0;
// X = diag(x)
#define RVL3VECTORTODIAGMX(x,X)	X[0] = x[0]; X[4] = x[1]; X[8] = x[2]; X[1] = X[2] = X[3] = X[5] = X[6] = X[7] = 0.0;
// X = diag([d1 d2 d3]')
#define RVLDIAGMX3(d1, d2, d3, X)	X[0] = d1; X[4] = d2; X[8] = d3; X[1] = X[2] = X[3] = X[5] = X[6] = X[7] = 0.0;
// element in i-th row and j-th column of matrix Mx with nCol columns
#define RVLMXEL(Mx, nCols, i, j)		Mx[nCols * i + j]
// Tgt = Src1(3x1) + Src2(3x1)
#define RVLSUM3VECTORS(Src1, Src2, Tgt)	Tgt[0] = Src1[0] + Src2[0]; Tgt[1] = Src1[1] + Src2[1]; Tgt[2] = Src1[2] + Src2[2]; 
// Tgt = Src1(3x1) - Src2(3x1)
#define RVLDIF3VECTORS(Src1, Src2, Tgt)	Tgt[0] = Src1[0] - Src2[0]; Tgt[1] = Src1[1] - Src2[1]; Tgt[2] = Src1[2] - Src2[2]; 
// Tgt = a * Src(3x1)
#define RVLSCALE3VECTOR(Src, a, Tgt)	Tgt[0] = a * Src[0]; Tgt[1] = a * Src[1]; Tgt[2] = a * Src[2]; 
// Tgt = Src(3x1) / a
#define RVLSCALE3VECTOR2(Src, a, Tgt)	Tgt[0] = Src[0] / a; Tgt[1] = Src[1] / a; Tgt[2] = Src[2] / a; 
// dot product of i-th row of A(3x3) and j-th column of B(3x3)
#define RVLMULROWCOL3(A,B,i,j)	(A[3*i+0]*B[3*0+j] + A[3*i+1]*B[3*1+j] + A[3*i+2]*B[3*2+j])
// dot product of i-th row of A(3x3) and j-th row of B(3x3)
#define RVLMULROWROW3(A,B,i,j)	(A[3*i+0]*B[3*j+0] + A[3*i+1]*B[3*j+1] + A[3*i+2]*B[3*j+2])
// dot product of i-th column of A(3x3) and j-th column of B(3x3)
#define RVLMULCOLCOL3(A,B,i,j)	(A[3*0+i]*B[3*0+j] + A[3*1+i]*B[3*1+j] + A[3*2+i]*B[3*2+j])
// y = A(3x3) * x(3x1)
#define RVLMULMX3X3VECT(A, x, y)	y[0] = A[0]*x[0] + A[1]*x[1] + A[2]*x[2]; y[1] = A[3]*x[0] + A[4]*x[1] + A[5]*x[2]; y[2] = A[6]*x[0] + A[7]*x[1] + A[8]*x[2];
// y = A(3x3)' * x(3x1)
#define RVLMULMX3X3TVECT(A, x, y)	y[0] = A[0]*x[0] + A[3]*x[1] + A[6]*x[2]; y[1] = A[1]*x[0] + A[4]*x[1] + A[7]*x[2]; y[2] = A[2]*x[0] + A[5]*x[1] + A[8]*x[2];
// invt = -R(3x3)' * t(3x1)
#define RVLINVTRANSL(R, t, invt)	invt[0] = -R[0]*t[0] - R[3]*t[1] - R[6]*t[2]; invt[1] = -R[1]*t[0] - R[4]*t[1] - R[7]*t[2]; invt[2] = -R[2]*t[0] - R[5]*t[1] - R[8]*t[2];
// y = A(3x3) * x(3x1), where A is a simetric matrix with only diagonal + upper triangle defined
#define RVLMULCOV3VECT(A, x, y)		y[0] = A[0]*x[0] + A[1]*x[1] + A[2]*x[2]; y[1] = A[1]*x[0] + A[4]*x[1] + A[5]*x[2]; y[2] = A[2]*x[0] + A[5]*x[1] + A[8]*x[2];
// y = min(x(3x1))
#define RVL3DVECTORMIN(x, y)	{if(x[0] <= x[1]) {if(x[0] <= x[2]) y = x[0]; else y = x[2];} else {if(x[1] <= x[2]) y = x[1]; else y = x[2];}}
// y = A(3x3) * j-th column of B(3x3)
#define RVLMULMXCOL3(A,B,j,y)	y[0] = A[0] * B[j] + A[1] * B[3+j] + A[2] * B[6+j]; y[1] = A[3] * B[j] + A[4] * B[3+j] + A[5] * B[6+j]; y[2] = A[6] * B[j] + A[7] * B[3+j] + A[8] * B[6+j]; 
// C = A(3x3)*B(3x3)
#define RVLMXMUL3X3(A,B,C)\
{\
	RVLMXEL(C, 3, 0, 0) = RVLMULROWCOL3(A,B,0,0);\
	RVLMXEL(C, 3, 0, 1) = RVLMULROWCOL3(A,B,0,1);\
	RVLMXEL(C, 3, 0, 2) = RVLMULROWCOL3(A,B,0,2);\
	RVLMXEL(C, 3, 1, 0) = RVLMULROWCOL3(A,B,1,0);\
	RVLMXEL(C, 3, 1, 1) = RVLMULROWCOL3(A,B,1,1);\
	RVLMXEL(C, 3, 1, 2) = RVLMULROWCOL3(A,B,1,2);\
	RVLMXEL(C, 3, 2, 0) = RVLMULROWCOL3(A,B,2,0);\
	RVLMXEL(C, 3, 2, 1) = RVLMULROWCOL3(A,B,2,1);\
	RVLMXEL(C, 3, 2, 2) = RVLMULROWCOL3(A,B,2,2);\
}
// C = A(3x3)*B'(3x3)
#define RVLMXMUL3X3T2(A,B,C)\
{\
	RVLMXEL(C, 3, 0, 0) = RVLMULROWROW3(A,B,0,0);\
	RVLMXEL(C, 3, 0, 1) = RVLMULROWROW3(A,B,0,1);\
	RVLMXEL(C, 3, 0, 2) = RVLMULROWROW3(A,B,0,2);\
	RVLMXEL(C, 3, 1, 0) = RVLMULROWROW3(A,B,1,0);\
	RVLMXEL(C, 3, 1, 1) = RVLMULROWROW3(A,B,1,1);\
	RVLMXEL(C, 3, 1, 2) = RVLMULROWROW3(A,B,1,2);\
	RVLMXEL(C, 3, 2, 0) = RVLMULROWROW3(A,B,2,0);\
	RVLMXEL(C, 3, 2, 1) = RVLMULROWROW3(A,B,2,1);\
	RVLMXEL(C, 3, 2, 2) = RVLMULROWROW3(A,B,2,2);\
}
// C = A'(3x3)*B(3x3)
#define RVLMXMUL3X3T1(A,B,C)\
{\
	RVLMXEL(C, 3, 0, 0) = RVLMULCOLCOL3(A,B,0,0);\
	RVLMXEL(C, 3, 0, 1) = RVLMULCOLCOL3(A,B,0,1);\
	RVLMXEL(C, 3, 0, 2) = RVLMULCOLCOL3(A,B,0,2);\
	RVLMXEL(C, 3, 1, 0) = RVLMULCOLCOL3(A,B,1,0);\
	RVLMXEL(C, 3, 1, 1) = RVLMULCOLCOL3(A,B,1,1);\
	RVLMXEL(C, 3, 1, 2) = RVLMULCOLCOL3(A,B,1,2);\
	RVLMXEL(C, 3, 2, 0) = RVLMULCOLCOL3(A,B,2,0);\
	RVLMXEL(C, 3, 2, 1) = RVLMULCOLCOL3(A,B,2,1);\
	RVLMXEL(C, 3, 2, 2) = RVLMULCOLCOL3(A,B,2,2);\
}
// Y = C(3x3)*J(3x3)'	(C is simmetric)
#define RVLMULCOV3MX3X3T(C, J, Y)\
{\
	RVLMXEL(Y, 3, 0, 0) = C[0]*J[0] + C[1]*J[1] + C[2]*J[2];\
	RVLMXEL(Y, 3, 0, 1) = C[0]*J[3] + C[1]*J[4] + C[2]*J[5];\
	RVLMXEL(Y, 3, 0, 2) = C[0]*J[6] + C[1]*J[7] + C[2]*J[8];\
	RVLMXEL(Y, 3, 1, 0) = C[1]*J[0] + C[4]*J[1] + C[5]*J[2];\
	RVLMXEL(Y, 3, 1, 1) = C[1]*J[3] + C[4]*J[4] + C[5]*J[5];\
	RVLMXEL(Y, 3, 1, 2) = C[1]*J[6] + C[4]*J[7] + C[5]*J[8];\
	RVLMXEL(Y, 3, 2, 0) = C[2]*J[0] + C[5]*J[1] + C[8]*J[2];\
	RVLMXEL(Y, 3, 2, 1) = C[2]*J[3] + C[5]*J[4] + C[8]*J[5];\
	RVLMXEL(Y, 3, 2, 2) = C[2]*J[6] + C[5]*J[7] + C[8]*J[8];\
}
#define RVLCOMPLETESIMMX3(A)\
{\
	A[3*1+0] = A[3*0+1];\
	A[3*2+0] = A[3*0+2];\
	A[3*2+1] = A[3*1+2];\
}
// Y = A(3x3)*B(3x3)	(only diagonal + upper triangle are computed)
#define RVLMULMX3X3UT(A, B, Y)\
{\
	Y[3*0+0] = A[3*0+0] * B[3*0+0] + A[3*0+1] * B[3*1+0] + A[3*0+2] * B[3*2+0];\
	Y[3*0+1] = A[3*1+0] * B[3*0+0] + A[3*1+1] * B[3*1+0] + A[3*1+2] * B[3*2+0];\
	Y[3*0+2] = A[3*2+0] * B[3*0+0] + A[3*2+1] * B[3*1+0] + A[3*2+2] * B[3*2+0];\
	Y[3*1+1] = A[3*1+0] * B[3*0+1] + A[3*1+1] * B[3*1+1] + A[3*1+2] * B[3*2+1];\
	Y[3*1+2] = A[3*2+0] * B[3*0+1] + A[3*2+1] * B[3*1+1] + A[3*2+2] * B[3*2+1];\
	Y[3*2+2] = A[3*2+0] * B[3*0+2] + A[3*2+1] * B[3*1+2] + A[3*2+2] * B[3*2+2];\
}
// COut = J(3x3)*C(3x3)*J(3x3)'	(C is simmetric; only diagonal + upper triangle are computed)
#define RVLCOV3DTRANSF(CIn, J, COut, Tmp)\
{\
	RVLMULCOV3MX3X3T(CIn, J, Tmp)\
	RVLMULMX3X3UT(J, Tmp, COut)\
}
// x(3x1)'*y(3x1)
#define RVLDOTPRODUCT3(x, y)	(x[0]*y[0]+x[1]*y[1]+x[2]*y[2])
// z = x(3x1) x y(3x1)
#define RVLCROSSPRODUCT3(x, y, z)		z[0] = x[1] * y[2] - x[2] * y[1];z[1] = x[2] * y[0] - x[0] * y[2];z[2] = x[0] * y[1] - x[1] * y[0];
// normalize vector x(3x1)
#define RVLNORM3(x, len)	{len = sqrt(RVLDOTPRODUCT3(x, x)); RVLSCALE3VECTOR2(x, len, x);}
// A = x(3x1) * y(3x1)'
#define RVLMULVECT3VECT3T(x, y, A)\
{\
	A[3*0+0] = x[0] * y[0]; A[3*0+1] = x[0] * y[1]; A[3*0+2] = x[0] * y[2];\
	A[3*1+0] = x[1] * y[0]; A[3*1+1] = x[1] * y[1]; A[3*1+2] = x[1] * y[2];\
	A[3*2+0] = x[2] * y[0]; A[3*2+1] = x[2] * y[1]; A[3*2+2] = x[2] * y[2];\
}
// x(3x1) * j-th column of A(3x3)
#define RVLMULVECTORCOL3(x, A, j)	(x[0]*A[3*0+j]+x[1]*A[3*1+j]+x[2]*A[3*2+j])
#define RVLCOVMX3BBVOLUME(C)	(C[3*0+1]*(2.0*C[3*0+2]*C[3*1+2] - C[3*2+2]*C[3*0+1]) - C[3*1+1]*C[3*0+2]*C[3*0+2] + C[3*0+0]*(C[3*1+1]*C[3*2+2] - C[3*1+2]*C[3*1+2]))
// invC = inv(C) (C is simmetric; only diagonal + upper triangle are computed)
#define RVLINVCOV3(C, invC, detC)\
{\
	detC = 2.0*C[5]*C[1]*C[2] - C[8]*C[1]*C[1] - C[4]*C[2]*C[2] - C[0]*(C[5]*C[5] - C[4]*C[8]);\
	invC[0] = (C[4]*C[8] - C[5]*C[5]) / detC;\
	invC[1] = (C[2]*C[5] - C[1]*C[8]) / detC;\
	invC[2] = (C[1]*C[5] - C[2]*C[4]) / detC;\
	invC[4] = (C[0]*C[8] - C[2]*C[2]) / detC;\
	invC[5] = (C[1]*C[2] - C[0]*C[5]) / detC;\
	invC[8] = (C[0]*C[4] - C[1]*C[1]) / detC;\
}
// return J(1x3)*C(3x3)*J(1x3)'
#define RVLCOV3DTRANSFTO1D(C, J)	(C[0]*J[0]*J[0] + 2*C[1]*J[0]*J[1] + 2*C[2]*J[0]*J[2] + C[4]*J[1]*J[1] + 2*C[5]*J[1]*J[2] + C[8]*J[2]*J[2])
#define RVLMIN(x, y)	(x <= y ? x : y)
#define RVLMAX(x, y)	(x >= y ? x : y)
// R = [cs, -sn, 0;
//		sn,  cs, 0;
//		0,   0,  1]
#define RVLROTZ(cs, sn, R)\
{\
	RVLMXEL(R, 3, 0, 0) = cs;\
	RVLMXEL(R, 3, 0, 1) = -sn;\
	RVLMXEL(R, 3, 0, 2) = 0.0;\
	RVLMXEL(R, 3, 1, 0) = sn;\
	RVLMXEL(R, 3, 1, 1) = cs;\
	RVLMXEL(R, 3, 1, 2) = 0.0;\
	RVLMXEL(R, 3, 2, 0) = 0.0;\
	RVLMXEL(R, 3, 2, 1) = 0.0;\
	RVLMXEL(R, 3, 2, 2) = 1.0;\
}
// V(3x1) = R(3x3) OX X ie[1 0 0]
#define RVLOX_X(R,V)\
{\
	V[0] = R[8] * R[4] - R[5] * R[7];\
	V[1] = R[6] * R[5] - R[3] * R[8];\
	V[2] = R[7] * R[3] - R[4] * R[6];\
}
// V(3x1) = R(3x3) OX Z ie[0 0 1]
#define RVLOX_Z(R,V)\
{\
	V[0] = R[5] * R[1] - R[2] * R[4];\
	V[1] = R[3] * R[2] - R[0] * R[5];\
	V[2] = R[4] * R[0] - R[1] * R[3];\
}
// C(3x3) = A(3x1) * B'(3x1)
#define RVLVECMUL3X1T2(A,B,C)\
{\
	RVLMXEL(C, 3, 0, 0) = A[0] * B[0];\
	RVLMXEL(C, 3, 0, 1) = A[0] * B[1];\
	RVLMXEL(C, 3, 0, 2) = A[0] * B[2];\
	RVLMXEL(C, 3, 1, 0) = A[1] * B[0];\
	RVLMXEL(C, 3, 1, 1) = A[1] * B[1];\
	RVLMXEL(C, 3, 1, 2) = A[1] * B[2];\
	RVLMXEL(C, 3, 2, 0) = A[2] * B[0];\
	RVLMXEL(C, 3, 2, 1) = A[2] * B[1];\
	RVLMXEL(C, 3, 2, 2) = A[2] * B[2];\
}
// T(R, t) = T(R1, t1) * T(R2, t2)
#define RVLCOMPTRANSF3D(R1, t1, R2, t2, R, t)\
{\
	RVLMXMUL3X3(R1, R2, R)\
	RVLMULMX3X3VECT(R1, t2, t)\
	RVLSUM3VECTORS(t, t1, t)\
}
// R = R1 * R2	(rotations around z-axis)
#define RVLCOMPROT3D3DOF(R1, R2, R)\
{\
	R[0] = R1[0]*R2[0]+R1[1]*R2[3];\
	R[1] = R1[0]*R2[1]+R1[1]*R2[4];\
	R[2] = 0.0;\
	R[3] = R1[3]*R2[0]+R1[4]*R2[3];\
	R[4] = R1[3]*R2[1]+R1[4]*R2[4];\
	R[5] = 0.0;\
	R[6] = 0.0;\
	R[7] = 0.0;\
	R[8] = 1.0;\
}
// T(R, t) = T(R1, t1) * T(R2, t2)	(3DOF transformations)
#define RVLCOMPTRANSF3D3DOF(R1, t1, R2, t2, R, t)\
{\
	RVLCOMPROT3D3DOF(R1, R2, R)\
	t[0] = R1[0]*t2[0]+R1[1]*t2[1]+t1[0];\
	t[1] = R1[3]*t2[0]+R1[4]*t2[1]+t1[1];\
	t[2] = 0.0;\
}
// T(R, t) = inv(T(R1, T1)) * T(R2, T2)
#define RVLCOMPTRANSF3DWITHINV(R1, t1, R2, t2, R, t, tmp3x1)\
{\
	RVLMXMUL3X3T1(R1, R2, R)\
	RVLDIF3VECTORS(t2, t1, tmp3x1)\
	RVLMULMX3X3TVECT(R1, tmp3x1, t)\
}
// T(RTgt, tTgt) = inv(T(RSrc, tSrc))
#define RVLINVTRANSF3D(RSrc, tSrc, RTgt, tTgt)\
{\
	RVLCOPYMX3X3T(RSrc, RTgt)\
	RVLINVTRANSL(RSrc, tSrc, tTgt)\
}
// T(RTgt, tTgt) = inv(T(RSrc, tSrc))	(3DOF transformations)
#define RVLINVTRANSF3D3DOF(RSrc, tSrc, RTgt, tTgt)\
{\
	RTgt[0] = RSrc[0];\
	RTgt[1] = RSrc[3];\
	RTgt[2] = 0.0;\
	RTgt[3] = RSrc[1];\
	RTgt[4] = RSrc[4];\
	RTgt[5] = 0.0;\
	RTgt[6] = 0.0;\
	RTgt[7] = 0.0;\
	RTgt[8] = 1.0;\
	tTgt[0] = - RSrc[0]*tSrc[0] - RSrc[3]*tSrc[1];\
	tTgt[1] = RSrc[3]*tSrc[0] - RSrc[0]*tSrc[1];\
	tTgt[2] = 0.0;\
}
// Tgt = Src(2x2)
#define RVLCOPYMX2X2(Src, Tgt)	Tgt[0] = Src[0]; Tgt[1] = Src[1]; Tgt[2] = Src[2]; Tgt[3] = Src[3];
// C = A(2x2)*B(2x2)
#define RVLMXMUL2X2(A,B,C)\
{\
	RVLMXEL(C, 2, 0, 0) = RVLMXEL(A, 2, 0, 0)*RVLMXEL(B, 2, 0, 0)+RVLMXEL(A, 2, 0, 1)*RVLMXEL(B, 2, 1, 0);\
	RVLMXEL(C, 2, 0, 1) = RVLMXEL(A, 2, 0, 0)*RVLMXEL(B, 2, 0, 1)+RVLMXEL(A, 2, 0, 1)*RVLMXEL(B, 2, 1, 1);\
	RVLMXEL(C, 2, 1, 0) = RVLMXEL(A, 2, 1, 0)*RVLMXEL(B, 2, 0, 0)+RVLMXEL(A, 2, 1, 1)*RVLMXEL(B, 2, 1, 0);\
	RVLMXEL(C, 2, 1, 1) = RVLMXEL(A, 2, 1, 0)*RVLMXEL(B, 2, 0, 1)+RVLMXEL(A, 2, 1, 1)*RVLMXEL(B, 2, 1, 1);\
}
// y = det(C(2x2)) (C is simmetric)
#define RVLDET2(C)	(C[0]*C[3] - C[1]*C[1])
// y = x(2x1)' * inv(C(2x2)) * x (C is simmetric)
#define RVLMAHDIST2(x, C, detC)	((C[3]*x[0]*x[0] - 2.0*C[1]*x[0]*x[1] + C[0]*x[1]*x[1]) / detC)
// COut = J(2x2)*C(2x2)*J(2x2)'	(C is simmetric; only diagonal + upper triangle are computed)
#define RVLCOV2DTRANSF(C, J, COut)\
{\
	COut[0] = C[0]*J[0]*J[0] + 2*C[1]*J[0]*J[1] + C[3]*J[1]*J[1];\
	COut[1] = J[2]*(C[0]*J[0] + C[1]*J[1]) + J[3]*(C[1]*J[0] + C[3]*J[1]);\
	COut[3] = C[0]*J[2]*J[2] + 2*C[1]*J[2]*J[3] + C[3]*J[3]*J[3];\
}

struct PIX_ARRAY
{
	int Width;
	int Height;
	int nPixBytes;
	unsigned char *pPix;
	BOOL bOwnData;
	BOOL bColor;
};

struct RVL2DMOMENTS
{
	int n;
	double S[2], S2[4];
};

struct RVL3DMOMENTS
{
	int n;
	double S[3], S2[9];
};

struct RVL2DOBB
{
	double u0, v0, cs, sn, w, h;
};

struct RVLHASHTABLE_ENTRY
{
	DWORD Data;
	RVLHASHTABLE_ENTRY *pNextEntry;
};

struct RVLRANDPDFLT
{
	int min;
	int max;
	int n;
	double nrm;
	double dval;
	double *LT;
};

struct RVLPDF_BILATERAL_GAUSSIAN
{
	double mean;
	double sigN;
	double sigP;
	double k;
};

int Max(int *x, int n);
int Min(int *x, int n);
double Max(double *x, int n);
double Min(double *x, int n);
int Max(int *x, int n, int &j);
int Min(int *x, int n, int &j);
double Max(double *x, int n, int &j);
float Max(float *x, int n, int &j);
double Min(double *x, int n, int &j);
float Min(float *x, int n, int &j);
BOOL TwoEqsTwoVars(double a11, double a12, double a21, double a22, double b1, double b2, double &x1, double &x2);
BOOL TwoEqsTwoVars(int a11, int a12, int a21, int a22, int b1, int b2, int &x1, int &x2);
BOOL TwoEqsTwoVars(int a11, int a12, int a21, int a22, int b1, int b2, double &x1, double &x2);
void GaussJordan3(double *A, double *B, double *X);
void MatrixMultiplication(double *A, double *B, double *Z, int n, int m);
void MatrixMultiplication(double *A, double *B, double *Z, int n1, int n2, int n3);
void MatrixMultiplicationT(double *A, double *B, double *Z, int n, int m);
void MatrixMultiplicationT(double *A, double *B, double *Z, int n1, int n2, int n3);
void MatrixMultiplicationT2(double *A, double *B, double *Z, int n1, int n2, int n3);
void rtUtilInit();
int rtsqrt(int x);
BYTE rtAngle90(int dx, int dy);
BYTE rtAngle180(int dx, int dy);
BYTE rtAngle180_2(int dx, int dy);
BYTE rtAngle360(int dx, int dy);
BYTE rtlog10(int x);
BYTE *CreateLogLookUpTable(int MinX, int MaxX, int CellSize, double Base);
void PanTiltRoll(double PanAngle, double TiltAngle, double RollAngle, double *Rotation/*, double *InvRotation*/);
void PanTiltRoll(double *Rot, double &PanAngle, double &TiltAngle, double &RollAngle);
void PTR2RPT(double PanPTR, double TiltPTR, double RollPTR,
	     double &PanRPT, double &TiltRPT, double &RollRPT);
void RPT2PTR(double PanRPT, double TiltRPT, double RollRPT,
	     double &PanPTR, double &TiltPTR, double &RollPTR);
void InverseRotation(double *InvRotation, double *Rotation);
void InverseTransform3D(double *InvRotation, double *InvTranslation, double *Rotation,
						double *Translation);
void RotFrom3DPoints(double *Rot, int *P11, int *P12, int *P21, int *P22);
void PTRFrom3DPoints(double &Pan, double &Tilt, double &Roll, double &r22, double &r33, int *P11, int *P12, int *P21, int *P22);
void RVLCombineTransform3D(double *Rot1, double *Trans1, double *Rot2, double *Trans2,
						double *Rot, double *Trans);
void InverseMatrix3(double *InvA, double *A);
BOOL InverseMatrix3(double *InvA, double *A, double MinDet);
BOOL InverseMatrix2(double *InvA, double *A, double MinDet);
void Sort(int *Key, int *Index, int n, BOOL bIndex = FALSE);
void Sort(double *Key, int *Index, int n, BOOL bIndex = FALSE);
void ANDDWArray(DWORD *X1, DWORD *X2, DWORD *Y, int n);
void ORDWArray(DWORD *X1, DWORD *X2, DWORD *Y, int n);
BOOL EqualDWArray(DWORD *X1, DWORD *X2, int n);
BOOL IsInAngle(int Angle, int AngleMin, int AngleMax);
void CrossProduct(double *X1, double *X2, double *Y);
void MultiplicationWithTransposed(double *A, double *Z, int n, int m);
BOOL GetMinEigenvector(double *A, double *q, double &d, int n);
void Robot2Camera(double *RotR, double *RotC);
BOOL Roots2(double *p, double *z);
void Roots3(double *p, double *z, BOOL *bReal);
void Roots4(double *p, double *z, BOOL *bReal);
void LinearTransform2D(double cs, double sn, double x0, double y0, double x, double y, 
					   double &p, double &q);
void InvLinearTransform3D(double *A, double *b, double *X, double *Y);
void LinearTransform2D(int dx, int dy, int len, int x0, int y0, int x, int y, int &p, int &q);
int Point2Line2DDist(int dx, int dy, int len, int x0, int y0, int x, int y);
void LinearTransform2D2(double cs, double sn, double x0, double y0, double x, double y, 
					   double &p, double &q);
void Get2DRot(double *Rot, double Alpha);
void Rotxz(double cs, double sn, double *X, double *Y);
void GetEllipseParams(double c11, double c12, double c22, double &R1, double &R2, 
					  double &tgPhi);
BOOL GetEllipseAxis(double c11, double c12, double c22, double &p1, double &p2, double &tgPhi);
void RVLQuickSort(short *Key, int *Index, int n);
//template <class Type>							// for Nyarko
//void RVLBubbleSort(CRVLMPtrChain *pInList,		// for Nyarko
//				   Type **OutArray,				// for Nyarko		
//				   BOOL descending = FALSE);	// for Nyarko
BOOL RVLEig2(double *C, double *eig);
BOOL RVLGetMaxEigVector2(double *C, double *eig, double *Veig);
void RVLEig3(double *C, double *eig, BOOL *bReal);
BOOL RVLGetMinEigVector3(double *C, double *eig, BOOL *bReal, double *Veig);
BOOL RVLGetMaxEigVector3(double *C, double *eig, BOOL *bReal, double *Veig);
void RVLGetCovMatrix2(RVL2DMOMENTS *pMoments, double *C, double *M);
void RVLGetCovMatrix3(RVL3DMOMENTS *pMoments, double *C, double *M);
void CreateUnitMatrix(double *A, int n);
void Sort3(double *X);
void RotCov2D(double *C, double cs, double sn, double *CRot);
void RotCov3D(double *C, double *Rot, double *CRot);
void RVLGetMainAxisOfCov3D(double *C, double *W3D);
BOOL RVLGetAxesOfCov3D(double *C, double *eig, double *VNrm, double *V, double *lV);
void RVLUnitMatrix(double *M, int n);
double RVLMahalanobisDistance2D(double *dX, double *C2D);
double RVLMahalanobisDistance3D(double *dX, double *C3D);
double RVLMahalanobisDistance(CvMat *dX, CvMat *C, CvMat *Tmp);
void RVLHSB2RGB(int h, int s, int k, int &r, int &g, int &b);
int *RVLGetsqrt_LookUpTable();
void RVLMomentsSum(RVL3DMOMENTS *pMoments1, RVL3DMOMENTS *pMoments2, RVL3DMOMENTS *pMomentsTgt);
BOOL RVLInsideTriangle(double Ax, double Ay, double Bx, double By, double Cx, double Cy, 
					   double Px, double Py);
void RVLGetCov3DFromCov2D(double *C2D, double *J, double *C3D);
void RVLGetCov2DFromCov3D(double *C3D, double *J, double *C2D);
double RVLGetCov1DFromCov3D(double *C3D, double *J);
double RVLGetCov1DFromCov2D(double *C2D, double *J);
void RVLCovTransf(double *CIn, double *J, int n, int m, double *COut);
void RVLVector2Matrix(double *X, int n, double *Y);
void RVLCompleteCov(double *C, int n);
void RVLCovTransf(CvMat *m_Csrc, 
				  CvMat *m_J,
				  CvMat *m_Ctgt,
				  CvMat *m_Tmp);
BOOL RVLGetEigVects3(double *C, 
				     double *eig, 
				     double *V);
void RVLPrintMatrix(FILE *fp, double *A, int n, int m);
double RVLGaussRand(double std);
double RVLGaussRandBM(double std);
void RVLResize(PIX_ARRAY *pPixArray, 
			   int Size);
void RVLRandomSelect(void **SrcArray, 
					 int n,
					 int m,
					 void **TgtArray);
/*void **RVLCreatePtrArray(CRVLMPtrChain *pSrcArray,
		  	             CRVLMem *pMem);*/
void RVLPrintCov(FILE *fp,
				 CvMat *C,
				 CvMat *W,
				 CvMat *U,
				 double k = 1.0);
void RVLPrintCov(FILE *fp,
				 double *C,
				 int n,
				 double k = 1.0);
void RVLInitMatrices();
/*void **RVLChainToArray(CRVLMPtrChain *pChain, 
					   CRVLMem *pMem,
					   int &n);*/
int RVLSqrDistance2LineSeg(	int *iU, 
							int *iU1, int *iU2);
void RVLGrayscaleToRGB(unsigned char *pPixSrc,
					   int w, int h,
					   unsigned char *pPixTgt);
void RVLCreateEmptyRegionGrowingMap(unsigned short *RegionGrowingMap,
									int w, int h,
									BYTE Empty,
									BYTE Border);
char *RVLCreateString(char *strIn);
void RVLCopyString(char *strSrc, char **pstrTgt, CRVLMem *pMem = NULL);
char *RVLCreateFileName(char *SrcFileName, 
						char *SrcExtension,
						int n,
						char *TgtExtension,
						CRVLMem *pMem = NULL);
void RVLCopyPixArray(PIX_ARRAY *SrcImage,
					 PIX_ARRAY *TgtImage);
void RVLCopyImageToPixArray(IplImage *pSrcImage,
							PIX_ARRAY *pTgtImage);
void Skew(double *V, double *M);
int* RVLWhereAbove(int *x, int n, double l, int &j);
int* RVLWhereAbove(float *x, int n, double l, int &j);
int* RVLWhereAbove(double *x, int n, double l, int &j);
void RVLSetFileNumber(char *FileName, char *Extension, int n);
int RVLGetFileNumber(char *FileName, char *Extension);
BOOL RVLGetNextFileName(char *FileName, char *Extension, int maxiSample);
void RVL3x3x3BlockMxTo6x6(double *PSrc, double *PTgt);
void RVL3DOFInvTransfUncert(double csalphaAB, 
							double snalphaAB,
							double txBA,
							double tyBA,
							double *C,
							double *invC);
void RVLZoom(IplImage *pSrcImage, IplImage *pTgtImage, int ZoomFactor);
double RVLTriangularPDF(double val,
						double Val0,
						double mindVal,
						double maxdVal);
double RVLBilateralGaussianPDF(	double val,
								double Val0,
								double mindVal,
								double maxdVal);
double RVLBilateralGaussianPDFAngle(double val,
									double Val0,
									double mindVal,
									double maxdVal);
double RVLLogBilateralGaussianPDF(	double val,
									double Val0,
									double mindVal,
									double maxdVal);
double RVLLogBilateralGaussianPDFAngle(double val,
									double Val0,
									double mindVal,
									double maxdVal);
double RVLBilateralGaussianPDF(	double val,
								void *vpParams);
void RVLRandPDFLT(	double PDF(double val, void *vpParams),
					void *vpParams_,
					double min,
					double max,
					double dval,
					double nrm,
					RVLRANDPDFLT *pLT);
double RVLRandPDF(RVLRANDPDFLT *pLT);
double RVLRandPDF(	double PDF(double val_, void *vpParams_),
					void *vpParams,
					double min,
					double max,
					double dval);

inline int rthypot(int x, int y)
{
	return rtsqrt(x * x + y * y);
}

inline int RVLRandom(int yMin, int yMax)
{
	return yMin + (yMax - yMin) * rand() / RAND_MAX;
}

inline double RVLRandom(double yMin, double yMax)
{
	return yMin + (yMax - yMin) * ((double)rand() / (double)RAND_MAX);
}

// Y(3 x 1) = A(3 x 3) * X(3 x 1)

inline void LinearTransform3D(double *A, double *X, double *Y)
{
	Y[0] = A[0 + 0 * 3] * X[0] + A[1 + 0 * 3] * X[1] + A[2 + 0 * 3] * X[2];
	Y[1] = A[0 + 1 * 3] * X[0] + A[1 + 1 * 3] * X[1] + A[2 + 1 * 3] * X[2];
	Y[2] = A[0 + 2 * 3] * X[0] + A[1 + 2 * 3] * X[1] + A[2 + 2 * 3] * X[2];
}

// Y(3 x 1) = A'(3 x 3) * X(3 x 1)

inline void LinearTransform3DT(double *A, double *X, double *Y)
{
	Y[0] = A[0 + 0 * 3] * X[0] + A[0 + 1 * 3] * X[1] + A[0 + 2 * 3] * X[2];
	Y[1] = A[1 + 0 * 3] * X[0] + A[1 + 1 * 3] * X[1] + A[1 + 2 * 3] * X[2];
	Y[2] = A[2 + 0 * 3] * X[0] + A[2 + 1 * 3] * X[1] + A[2 + 2 * 3] * X[2];
}

// Y(3 x 1) = A(3 x 3) * X(3 x 1) + b(3 x 1)

inline void LinearTransform3D(double *A, double *b, double *X, double *Y)
{
	Y[0] = A[0 + 0 * 3] * X[0] + A[1 + 0 * 3] * X[1] + A[2 + 0 * 3] * X[2] + b[0];
	Y[1] = A[0 + 1 * 3] * X[0] + A[1 + 1 * 3] * X[1] + A[2 + 1 * 3] * X[2] + b[1];
	Y[2] = A[0 + 2 * 3] * X[0] + A[1 + 2 * 3] * X[1] + A[2 + 2 * 3] * X[2] + b[2];
}

inline double RVLDotProduct(double *X1, double *X2)
{
	return X1[0] * X2[0] + X1[1] * X2[1] + X1[2] * X2[2];
}

inline double RVLGet3DVectorLen(double *V3D)
{
	return sqrt(V3D[0] * V3D[0] + V3D[1] * V3D[1] + V3D[2] * V3D[2]);
}

inline void RVLNormalize3DVector(double *V3D)
{
	double l = sqrt(V3D[0] * V3D[0] + V3D[1] * V3D[1] + V3D[2] * V3D[2]);

	V3D[0] /= l;
	V3D[1] /= l;
	V3D[2] /= l;
}

inline void RVLSum4D(double *X1, double *X2, double *Y)
{
	Y[0] = X1[0] + X2[0];
	Y[1] = X1[1] + X2[1];
	Y[2] = X1[2] + X2[2];
	Y[3] = X1[3] + X2[3];
}

inline void RVLSum3D(double *X1, double *X2, double *Y)
{
	Y[0] = X1[0] + X2[0];
	Y[1] = X1[1] + X2[1];
	Y[2] = X1[2] + X2[2];
}

inline void RVLSum3DWeighted(double *X1, double *X2, double w, double *Y)
{
	Y[0] = X1[0] + w * X2[0];
	Y[1] = X1[1] + w * X2[1];
	Y[2] = X1[2] + w * X2[2];
}

inline void RVLDif4D(double *X1, double *X2, double *Y)
{
	Y[0] = X1[0] - X2[0];
	Y[1] = X1[1] - X2[1];
	Y[2] = X1[2] - X2[2];
	Y[3] = X1[3] - X2[3];
}

inline void RVLDif3D(double *X1, double *X2, double *Y)
{
	Y[0] = X1[0] - X2[0];
	Y[1] = X1[1] - X2[1];
	Y[2] = X1[2] - X2[2];
}

inline void RVLDif2D(double *X1, double *X2, double *Y)
{
	Y[0] = X1[0] - X2[0];
	Y[1] = X1[1] - X2[1];
}

inline void RVLBoundingBoxUpdate(double p, double q, double &minp, double &maxp, 
								 double &minq, double &maxq, BOOL &bFirst)
{
	if(bFirst)
	{
		bFirst = FALSE;

		minp = maxp = p;
		minq = maxq = q;
	}
	else
	{
		if(p < minp)
			minp = p;
		else if(p > maxp)
			maxp = p;

		if(q < minq)
			minq = q;
		else if(q > maxq)
			maxq = q;	
	}
};

inline void RVLBoundingBoxUpdate(int p, int q, int &minp, int &maxp, 
								 int &minq, int &maxq, BOOL &bFirst)
{
	if(bFirst)
	{
		bFirst = FALSE;

		minp = maxp = p;
		minq = maxq = q;
	}
	else
	{
		if(p < minp)
			minp = p;
		else if(p > maxp)
			maxp = p;

		if(q < minq)
			minq = q;
		else if(q > maxq)
			maxq = q;	
	}
};


inline void RVLAngleDiff(double phi1, double phi2, double &dphi)
{
	dphi = phi2 - phi1;

	if(dphi > PI)
		dphi -= 2.0 * PI;
	else if(dphi < -PI)
		dphi += 2.0 * PI;
};

// M(3x3) = X1(3x1) * X2(3x1)'

inline void RVLM3D(double *X1, double *X2, double *M)
{
	M[0*3+0] = X1[0] * X2[0];
	M[0*3+1] = X1[0] * X2[1];
	M[0*3+2] = X1[0] * X2[2];

	M[1*3+0] = X1[1] * X2[0];
	M[1*3+1] = X1[1] * X2[1];
	M[1*3+2] = X1[1] * X2[2];

	M[2*3+0] = X1[2] * X2[0];
	M[2*3+1] = X1[2] * X2[1];
	M[2*3+2] = X1[2] * X2[2];
}

// M(3x3) = M(3x3) + X1(3x1) * X2(3x1)'

inline void RVLUpdateM3D(double *X1, double *X2, double w, double *M)
{
	M[0*3+0] += (w * X1[0] * X2[0]);
	M[0*3+1] += (w * X1[0] * X2[1]);
	M[0*3+2] += (w * X1[0] * X2[2]);

	M[1*3+0] += (w * X1[1] * X2[0]);
	M[1*3+1] += (w * X1[1] * X2[1]);
	M[1*3+2] += (w * X1[1] * X2[2]);

	M[2*3+0] += (w * X1[2] * X2[0]);
	M[2*3+1] += (w * X1[2] * X2[1]);
	M[2*3+2] += (w * X1[2] * X2[2]);
}

inline CvSize RVLGetPixArraySize(PIX_ARRAY *pPixArray)
{
	return cvSize(pPixArray->Width, pPixArray->Height);
}

// created by Damir Filko
// adapted for general case by Robert Cupec
/*
template <class Type>
void RVLBubbleSort(CRVLMPtrChain *pInList,
				   Type **OutArray,				
				   BOOL descending = FALSE)
{
	//creating array for sorting purposes
	int n = pInList->m_nElements;
	if(OutArray == NULL)
		OutArray = new Type*[n];
	Type **ppElement = OutArray;
	Type *pElement;
	int i;

	pInList->Start();
	for(i = 0; i < n; i++)
	{
		pElement = (Type *)(pInList->GetNext());
		*(ppElement++) = pElement;
	}
	//Sortiranje - bubble sort
	Type* tempVoid;
	bool chg = true;
	while(chg)
	{
		chg = false;
		for(i = 0; i < n - 1; i++)
		{
			if (descending)
			{
				if (((Type *)OutArray[i + 1])->cost > 
					((Type *)OutArray[i])->cost)
				{
					tempVoid = OutArray[i];
					OutArray[i] = OutArray[i + 1];
					OutArray[i + 1] = tempVoid;

					chg = true;
				}
			}
			else
			{
				if (((Type *)OutArray[i + 1])->cost < 
					((Type *)OutArray[i])->cost)
				{
					tempVoid = OutArray[i];
					OutArray[i] = OutArray[i + 1];
					OutArray[i + 1] = tempVoid;

					chg = true;
				}
			}
		}
	}
}*/
/*
template <class Type>
void RVLResetFlags(CRVLMPtrChain *pObjectList,
				   DWORD Flag)
{
	Type *pObject;

	pObjectList->Start();

	while(pObjectList->m_pNext)
	{
		pObject = (Type *)(pObjectList->GetNext());

		pObject->m_Flags &= ~Flag;
	}
}*/

// Undersampling of image Src.
// w - width of Src; h - height of Src; both w and h must be multiples of 2
// The result is stored in Tgt.
// Tgt must be allocated beforehand.

template <class Type>
void RVLUnderSampleHalf(Type *Src,
						int w, int h,
						Type *Tgt)
{
	Type *pTgt = Tgt;
	Type *pSrc = Src;
	int wHalf = w / 2;
	int hHalf = h / 2;
	Type *pTgtEnd = Tgt + wHalf * hHalf;

	Type *pTgtRowEnd;

	for(pTgtRowEnd = Tgt + wHalf; pTgtRowEnd <= pTgtEnd; pTgtRowEnd += wHalf)
	{
		for(; pTgt < pTgtRowEnd; pTgt++, pSrc += 2)
			*pTgt = *pSrc;

		pSrc += w;
	}
}


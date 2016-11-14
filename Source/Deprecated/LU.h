#ifndef LU_H
#define LU_H
#include "Matrix.h"
class LU 
{
	public:
		LU() {;};
		Matrix extractUpperTriangularMatrix() const;
		Matrix extractLowerTriangularMatrix() const;

		Matrix Q;
		Matrix rowPermutationMat;
		double determinant;
};

Matrix LU::extractUpperTriangularMatrix() const {
	Matrix ut(Q.getNumRows(), Q.getNumColumns());
	ut.setAsIdentityMatrix();
	for (int row = 1; row <= Q.getNumRows(); row++) {
		for (int column = row+1; column <= Q.getNumColumns(); column++) {
			ut.set(row, column, Q.get(row,column));
		}
	}
	return ut;
}

Matrix LU::extractLowerTriangularMatrix() const {
	Matrix lt(Q.getNumRows(), Q.getNumColumns());
	lt.setAsIdentityMatrix();
	for (int column = 1; column <= Q.getNumColumns(); column++) {
		for (int row = column; row <= Q.getNumRows(); row++) {
			lt.set(row, column, Q.get(row,column));
		}
	}
	return lt;
}
#endif

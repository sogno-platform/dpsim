#ifndef MATRIX_H
#define MATRIX_H

//#include "LU.h"
#include "VectorJI.h"
#include <iostream>
#include <math.h>
#include <stdlib.h>

using std::cout;
using std::endl;

class LU;
class AcsMatrix;

class AcsMatrix 
{
	public:	
		AcsMatrix() { numRows = numColumns = 0; };
		AcsMatrix(int rows, int columns);
		void SetSize(int rows, int columns);
		bool isEqual(const AcsMatrix& b) const;
		void removeColumn(int column);		
		void removeRow(int row);
		void addRow();
		void addColumn();
		void addRow(const AcsMatrix& row);
		void addColumn(const AcsMatrix& column);		
		int getNumRows() const;		
		int getNumColumns() const;	
		double get(int row, int column) const;		
		void set(int row, int column, double value);		
		void clear();
		void setAsIdentityMatrix();		
		static AcsMatrix transposeMatrix(const AcsMatrix& input);		
		static AcsMatrix augmentMatrix(const AcsMatrix& matrix1, const AcsMatrix& matrix2);		
		void interchangeRows(int row1, int row2);
		static AcsMatrix addMatrices(const AcsMatrix& addend1, const AcsMatrix& addend2);		
		static AcsMatrix multiplyByScalar(const AcsMatrix& matrix, double factor);		
		static AcsMatrix subtractMatrices(const AcsMatrix& a, const AcsMatrix& b);		
		static AcsMatrix multiplyMatrices(const AcsMatrix& m1, const AcsMatrix& m2);		
		void setRow(int row, double* values);		
		AcsMatrix extractColumn(int column) const;	
		void replaceColumn(int column, const AcsMatrix& colMat);		
		AcsMatrix extractRow(int row) const;
		void replaceRow(int row, const AcsMatrix& rowMat);		
		void printMatrix() const;		
		static AcsMatrix computeInverse(const AcsMatrix& A, const LU& luFactored);		
		static AcsMatrix solveLinearSystem(const AcsMatrix& A, const AcsMatrix& b, const LU& luFactor);		
		AcsMatrix partitionMatrix(int afterWhichColumn); //returns RHS of partition, original matrix now contains LHS
		LU computeLUFactorization() const;
	private:
		int numRows;
		int numColumns;
		VectorJI<VectorJI<double> > rowVec;
};

class LU 
{
	public:
		LU() { Q = rowPermutationMat = AcsMatrix();};
		AcsMatrix extractUpperTriangularMatrix() const;
		AcsMatrix extractLowerTriangularMatrix() const;

		AcsMatrix Q;
		AcsMatrix rowPermutationMat;
		double determinant;
};

	
#endif

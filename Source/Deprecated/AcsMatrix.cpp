#include "AcsMatrix.h"

AcsMatrix LU::extractUpperTriangularMatrix() const {
	AcsMatrix ut(Q.getNumRows(), Q.getNumColumns());
	ut.setAsIdentityMatrix();
	for (int row = 1; row <= Q.getNumRows(); row++) {
		for (int column = row+1; column <= Q.getNumColumns(); column++) {
			ut.set(row, column, Q.get(row,column));
		}
	}
	return ut;
}

AcsMatrix LU::extractLowerTriangularMatrix() const {
	AcsMatrix lt(Q.getNumRows(), Q.getNumColumns());
	lt.setAsIdentityMatrix();
	for (int column = 1; column <= Q.getNumColumns(); column++) {
		for (int row = column; row <= Q.getNumRows(); row++) {
			lt.set(row, column, Q.get(row,column));
		}
	}
	return lt;
}

bool AcsMatrix::isEqual(const AcsMatrix& b) const {
	if (numRows != b.numRows || numColumns != b.numColumns) {
		return false;
	}
	for (int i = 1; i <= numRows; i++) {
		for (int j = 1; j <= numColumns; j++) {
			if (get(i,j) != b.get(i,j)) {
				return false;
			}
		}
	}
	return true;
}

AcsMatrix::AcsMatrix(int rows, int columns) {
	numRows = rows;
	numColumns = columns;
	for (int i = 0; i < numRows; i++) {
		VectorJI<double> newVec;
		newVec.setSize(numColumns);
		rowVec.add(newVec);
	}
	for (int i = 1; i <= numRows; i++) {
		for (int j = 1; j <= numColumns; j++) {
			this->set(i,j,0);
		}
	}
}


void AcsMatrix::SetSize(int rows, int columns) {
	numRows = rows;
	numColumns = columns;
	for (int i = 0; i < numRows; i++) {
		VectorJI<double> newVec;
		newVec.setSize(numColumns);
		rowVec.add(newVec);
	}
	for (int i = 1; i <= numRows; i++) {
		for (int j = 1; j <= numColumns; j++) {
			this->set(i,j,0);
		}
	}
}


void AcsMatrix::addRow() {
	addRow(AcsMatrix(1,numColumns));
}

void AcsMatrix::addColumn() {
	addColumn(AcsMatrix(numRows,1));
}

void AcsMatrix::clear() {
	for(int i=1; i<=numRows; i++)
		for(int j=1; j<=numColumns; j++)
			set(i,j,0.0);
}


void AcsMatrix::removeColumn(int column) {
	for (int i = 0; i < numRows; i++) {
		VectorJI<double> old = rowVec.elementAt(i);
		old.remove(column - 1);
		rowVec.setElementAt(old, i);
		//(rowVec.elementAt(i)).remove(column - 1);
	}
	numColumns --;
}

void AcsMatrix::removeRow(int row) {
	rowVec.remove(row - 1);
	numRows--;
}

void AcsMatrix::addRow(const AcsMatrix& row) {
	VectorJI<double> rowval = row.rowVec.elementAt(0);
	rowVec.add(numRows, rowval);
	numRows++;
}

void AcsMatrix::addColumn(const AcsMatrix& column) {
	for (int i = 0; i < numRows; i++) {
		VectorJI<double> old = rowVec.elementAt(i);
		old.add(column.get(i+1,1));
		rowVec.setElementAt(old, i);
//		((VectorJI<double>)rowVec.elementAt(i)).add(column.get(i+1,1));
	}
	numColumns++;
}

int AcsMatrix::getNumRows() const {
	return numRows;
}

int AcsMatrix::getNumColumns() const {
	return numColumns;
}

double AcsMatrix::get(int row, int column) const {
	return ((VectorJI<double>)(rowVec.elementAt(row-1))).elementAt(column-1);
}

void AcsMatrix::set(int row, int column, double value) {
	/*((VectorJI<double>)(rowVec.elementAt(row-1))).setElementAt(value, column-1);*/
	VectorJI<double> old = rowVec.elementAt(row - 1);
	old.setElementAt(value, column - 1);
	rowVec.setElementAt(old, row - 1);
}

void AcsMatrix::setAsIdentityMatrix() {
	if (numRows != numColumns) {
		cout << "Invalid dimensions in matrix calling Matrix.setAsIdentityMatrix() - matrix must be square." << endl;
		return;
	}
	
	for (int i = 1; i <= numRows; i++) {
		for (int j = 1; j <= numColumns; j++) {
			if (i == j) {
				this->set(i,j,1);
			} else {
				this->set(i,j,0);
			}
		}
	}
}

AcsMatrix AcsMatrix::transposeMatrix(const AcsMatrix& input) {
	AcsMatrix transpose(input.getNumColumns(), input.getNumRows());

	for (int i = 1; i <= transpose.numRows; i++) {
		for (int j = 1; j <= transpose.numColumns; j++) {
			transpose.set(i,j,input.get(j,i));
		}
	}
	
	return transpose;
}

AcsMatrix AcsMatrix::augmentMatrix(const AcsMatrix& matrix1, const AcsMatrix& matrix2) {
	if (matrix1.getNumRows() != matrix2.getNumRows()) {
		cout << "Error: attempting to augment two matrices with different numbers of rows." << endl;
		return AcsMatrix(0,0);
	}
	
	AcsMatrix augmented(matrix1.getNumRows(), matrix1.getNumColumns() + matrix2.getNumColumns());
	
	for (int i = 1; i <= matrix1.numRows; i++) {
		for (int j = 1; j <= matrix1.numColumns; j++) {
			augmented.set(i,j,matrix1.get(i,j));
		}
	}
	for (int i = 1; i <= matrix2.numRows; i++) {
		for (int j = 1; j <= matrix2.numColumns; j++) {
			augmented.set(i,j+matrix1.numColumns,matrix2.get(i,j));
		}
	}
	
	return augmented;
}

void AcsMatrix::interchangeRows(int row1, int row2) {
	VectorJI<double> temp = rowVec.get(row1-1);
	rowVec.setElementAt(rowVec.get(row2-1), row1-1);
	rowVec.setElementAt(temp, row2-1);
}

AcsMatrix AcsMatrix::addMatrices(const AcsMatrix& addend1, const AcsMatrix& addend2) {
	if (addend1.numColumns != addend2.numColumns || addend1.numRows != addend2.numRows) {
		cout << "Error: Cannot add matrices with incompatible dimensions." << endl;
		return AcsMatrix(0,0);
	}
	AcsMatrix sum(addend1.numRows, addend1.numColumns);
	for (int i = 1; i <= sum.numRows; i++) {
		for (int j = 1; j <= sum.numColumns; j++) {
			sum.set(i,j,addend1.get(i,j)+addend2.get(i,j));
		}
	}
	return sum;
}

AcsMatrix AcsMatrix::multiplyByScalar(const AcsMatrix& matrix, double factor) {
	AcsMatrix scaledMat(matrix.getNumRows(), matrix.getNumColumns());
	for (int row =1; row<=matrix.getNumRows(); row++) {
		for (int col = 1; col <= matrix.getNumColumns(); col++) {
			scaledMat.set(row,col,factor*matrix.get(row,col));
		}
	}
	return scaledMat;
}

AcsMatrix AcsMatrix::subtractMatrices(const AcsMatrix& a, const AcsMatrix& b) {
	if (a.numColumns != b.numColumns || a.numRows != b.numRows) {
		cout << "Error: Cannot subtract matrices with incompatible dimensions." << endl;
		return AcsMatrix(0,0);
	}
	AcsMatrix diff(a.numRows, a.numColumns);
	for (int i = 1; i <= diff.numRows; i++) {
		for (int j = 1; j <= diff.numColumns; j++) {
			diff.set(i,j,a.get(i,j)-b.get(i,j));
		}
	}
	return diff;
}

AcsMatrix AcsMatrix::multiplyMatrices(const AcsMatrix& m1, const AcsMatrix& m2) {
	if (m1.numColumns != m2.numRows) {
		cout << "Incompatible matrices requested for multiplication." << endl;
		return AcsMatrix(0,0);
	}
	
	AcsMatrix product(m1.numRows, m2.numColumns);
	for (int row = 1; row <= product.numRows; row++) {
		for (int col = 1; col <= product.numColumns; col++) {
			double sum = 0;
			for (int n = 1; n <= m1.numColumns; n++) {
				sum += m1.get(row,n)*m2.get(n,col);
			}
			product.set(row,col,sum);
		}
	}
	
	return product;
}

void AcsMatrix::setRow(int row, double* values) {
	for (int i = 1; i <= numColumns; i++) {
		this->set(row,i,values[i-1]);
	}
}

AcsMatrix AcsMatrix::extractColumn(int column) const {
	AcsMatrix columnMat(numRows,1);
	for (int i = 1; i <= numRows; i++) {
		columnMat.set(i,1,get(i,column));
	}
	return columnMat;
}

void AcsMatrix::replaceColumn(int column, const AcsMatrix& colMat) {
	for (int i = 1; i <= numRows; i++) {
		this->set(i,column,colMat.get(i,1));
	}
}

AcsMatrix AcsMatrix::extractRow(int row) const {
	VectorJI<double> rowExt = rowVec.get(row-1);
	AcsMatrix rowMat(1, numColumns);
	rowMat.rowVec.setElementAt(rowExt, 0);
	return rowMat;
}

void AcsMatrix::replaceRow(int row, const AcsMatrix& rowMat) {
	this->rowVec.setElementAt(rowMat.rowVec.get(0), row - 1);
}

void AcsMatrix::printMatrix() const {
	cout.precision(2);
	for (int j = 1; j <= numRows; j++) {
		cout << "[" << std::fixed << get(j,1);
		for (int i=2; i <= numColumns; i++) {
			cout << "\t" << std::fixed << get(j,i);
		}
		cout << "]\n";
	}
	cout << endl;
}

AcsMatrix AcsMatrix::computeInverse(const AcsMatrix& A, const LU& luFactored) {
	AcsMatrix identity (A.numRows, A.numColumns);
	identity.setAsIdentityMatrix();
	AcsMatrix inverse(A.numRows, A.numColumns);
	for (int col = 1; col <= A.numColumns; col++) {
		inverse.replaceColumn(col,AcsMatrix::solveLinearSystem(A,identity.extractColumn(col),luFactored));
	}
	return inverse;
}

AcsMatrix AcsMatrix::solveLinearSystem(const AcsMatrix& A, const AcsMatrix& b, const LU& luFactor) {
	AcsMatrix d = AcsMatrix::multiplyMatrices(luFactor.rowPermutationMat,b);
	AcsMatrix y(A.numRows,1);
	for (int k = 1; k <= A.numRows; k++) {
		double prodsum = 0;
		for (int i =1; i<=k-1; i++) {
			prodsum += luFactor.Q.get(k,i)*y.get(i,1);
		}
		y.set(k,1,(d.get(k,1)-prodsum)/luFactor.Q.get(k,k));
	}
	AcsMatrix x(A.numRows,1);
	for (int k = A.numRows; k >= 1; k--) {
		double prodsum = 0;
		for (int i=k+1; i <= A.numRows; i++) {
			prodsum += luFactor.Q.get(k,i)*x.get(i,1);
		}
		x.set(k,1, y.get(k,1) - prodsum);
	}
	return x;
}

//returns RHS of partition, original matrix now contains LHS
AcsMatrix AcsMatrix::partitionMatrix(int afterWhichColumn) {
	AcsMatrix RHS(numRows, numColumns - afterWhichColumn);
	for (int row = 1; row <= RHS.getNumRows(); row++) {
		for (int col=1; col <= RHS.getNumColumns(); col++) {
			RHS.set(row,col,get(row, afterWhichColumn+col));
		}
		VectorJI<double> old = rowVec.elementAt(row-1);
		old.setSize(afterWhichColumn);
		rowVec.setElementAt(old, row - 1);
		//((VectorJI<double>)(rowVec.elementAt(row-1))).setSize(afterWhichColumn);
	}
	numColumns = afterWhichColumn;
	return RHS;
}

LU AcsMatrix::computeLUFactorization() const {
	AcsMatrix identity(numRows, numColumns);
	identity.setAsIdentityMatrix();
	AcsMatrix augmented = AcsMatrix::augmentMatrix(*this,identity);
	double det = 1;
	for (int j =1; j <= numRows; j++) {
		for (int k = j; k <= numRows; k++) {
			double prodsum = 0;
			for (int i=1; i <=j-1; i++) {
				prodsum += augmented.get(k,i) * augmented.get(i,j);
			}
			augmented.set(k,j,augmented.get(k,j)-prodsum);
		}
		
		int rowPivot=j;
		double currMax = abs(augmented.get(j,j));
		for (int i = j; i <= numRows; i++) {
			if (abs(augmented.get(i,j)) > currMax) {
				currMax = abs(augmented.get(i,j));
				rowPivot = i;
			}
		}
		if (currMax == 0) {
			cout << "Singular matrix - cannot compute LU factorization." << endl;
			return LU();
		}
		if (rowPivot > j) {
			augmented.interchangeRows(rowPivot,j);
			det *= -1;
		}
		
		for (int k = j+1; k <= numRows; k++) {
			double prodsum = 0;
			for (int i =1; i <=j-1; i++) {
				prodsum += augmented.get(j,i)*augmented.get(i,k);
			}
			double newVal = (augmented.get(j,k) - prodsum)/augmented.get(j,j);
			augmented.set(j,k,newVal);
		}
		det *= augmented.get(j,j);			
	}
	LU returnLU;
	returnLU.rowPermutationMat = augmented.partitionMatrix(numRows);
	returnLU.Q = augmented;
	returnLU.determinant = det;
	return returnLU;
}
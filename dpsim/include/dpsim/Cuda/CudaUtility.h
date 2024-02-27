/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <iostream>
#include <utility>

#include <cuda_runtime.h>
#include <cusparse_v2.h>

//Error Handling
#define CUDA_ENABLE_ERROR_CHECK

#ifdef CUDA_ENABLE_ERROR_CHECK
#define CUDA_CHECK_ERROR(CUDA_CALL)                                            \
  {                                                                            \
    cudaError_t code = (cudaError_t)CUDA_CALL;                                 \
    if (code != cudaSuccess) {                                                 \
      printf("CUDA Error: %s\n", cudaGetErrorString(code));                    \
    }                                                                          \
  }
#else
#define CUDA_CHECK_ERROR(CUDA_CALL) CUDA_CALL
#endif

/**
 *   * copybackAndPrint()
 *        Copies memory back and puts data into "stream"
 *   * class Vector
 *        works like std::vector, however with internal buffer on device-memory
 *   * class CudaMatrix
 *        Representation of a row-major sparse matrtix on device-memory (compatible with eigen)
 */

namespace DPsim {
namespace cuda {

template <typename T>
void copybackAndPrint(std::ostream &stream, const char *msg, T *ptr, int n) {
  std::vector<T> buffer(n);
  CUDA_CHECK_ERROR(
      cudaMemcpy(buffer.data(), ptr, n * sizeof(T), cudaMemcpyDeviceToHost));
  stream << msg;
  for (T d : buffer) {
    std::cout << d << ' ';
  }
  stream << std::endl;
}

template <typename T> class Vector {
public:
  Vector(size_t preAlloc) : buf(nullptr), n(preAlloc) {
    if (n > 0) {
      CUDA_CHECK_ERROR(cudaMalloc(&buf, preAlloc * sizeof(T)));
    }
  }

  Vector(Vector &) = delete;
  Vector(Vector &&other) {
    std::swap(buf, other.buf);
    std::swap(n, other.n);
  }

  ~Vector() {
    if (n > 0) {
      CUDA_CHECK_ERROR(cudaFree(buf));
    }
  }

  Vector &operator=(const Vector &) = delete;
  Vector &operator=(Vector &&other) {
    std::swap(buf, other.buf);
    std::swap(n, other.n);
    return *this;
  }

  T operator[](size_t pos) {
    T element;
    CUDA_CHECK_ERROR(
        cudaMemcpy(&element, &buf[pos], sizeof(T), cudaMemcpyDeviceToHost));
    return element;
  }

  T *data() const { return buf; }

  class iterator : public std::iterator<std::random_access_iterator_tag, T> {
  public:
    iterator(T *buf, long num) : buf(buf), pos(num) {}
    iterator &operator++() {
      pos++;
      return *this;
    }
    iterator operator++(int) {
      iterator retval = *this;
      ++(*this);
      return retval;
    }
    iterator &operator--() {
      pos--;
      return *this;
    }
    iterator operator--(int) {
      iterator retval = *this;
      --(*this);
      return retval;
    }
    bool operator==(iterator other) const {
      return buf == other.buf && pos == other.pos;
    }
    bool operator!=(iterator other) const { return !(*this == other); }
    T operator*() const {
      T element;
      CUDA_CHECK_ERROR(
          cudaMemcpy(&element, &buf[pos], sizeof(T), cudaMemcpyDeviceToHost));
      return element;
    }

  private:
    size_t pos;
    T *buf;
  };

  iterator begin() { return iterator(buf, 0); }
  iterator end() { return iterator(buf, n - 1); }

private:
  T *buf;
  size_t n;
};

// Matrix (CSR)
template <typename ValueType, typename IndexType> struct CudaMatrix {

  CudaMatrix(const Eigen::SparseMatrix<ValueType, Eigen::RowMajor> &mat,
             int dim)
      : dim(dim), non_zero(mat.nonZeros()), row(dim + 1), col(mat.nonZeros()),
        val(mat.nonZeros()) {

    // Copy Matrix (Host -> Device)
    CUDA_CHECK_ERROR(cudaMemcpy(row.data(), mat.outerIndexPtr(),
                                (dim + 1) * sizeof(int),
                                cudaMemcpyHostToDevice));
    CUDA_CHECK_ERROR(cudaMemcpy(col.data(), mat.innerIndexPtr(),
                                non_zero * sizeof(int),
                                cudaMemcpyHostToDevice));
    CUDA_CHECK_ERROR(cudaMemcpy(val.data(), mat.valuePtr(),
                                non_zero * sizeof(double),
                                cudaMemcpyHostToDevice));
  }

  //Matrix Data
  const int dim;
  const int non_zero;
  cuda::Vector<IndexType> row;
  cuda::Vector<IndexType> col;
  cuda::Vector<ValueType> val;

  friend std::ostream &
  operator<<(std::ostream &os,
             const DPsim::cuda::CudaMatrix<ValueType, IndexType> &mat) {
    //Copy back
    std::vector<double> bufferVal(mat.non_zero);
    std::vector<int> bufferCol(mat.non_zero);
    std::vector<int> bufferRow(mat.dim);
    CUDA_CHECK_ERROR(cudaMemcpy(bufferVal.data(), mat.val.data(),
                                mat.non_zero * sizeof(double),
                                cudaMemcpyDeviceToHost));
    CUDA_CHECK_ERROR(cudaMemcpy(bufferCol.data(), mat.col.data(),
                                mat.non_zero * sizeof(int),
                                cudaMemcpyDeviceToHost));
    CUDA_CHECK_ERROR(cudaMemcpy(bufferRow.data(), mat.row.data(),
                                mat.dim * sizeof(int), cudaMemcpyDeviceToHost));

    //Print Sparse Structures (Eigen-Format)
    os << "Nonzero entries:\n";
    for (int i = 0; i < mat.non_zero; i++) {
      os << '(' << bufferVal[i] << ',' << bufferCol[i] << ") ";
    }
    os << "\n\n";
    os << "Outer pointers:\n";
    for (auto i : bufferRow) {
      os << i << ' ';
    }
    os << " $\n";
    // TODO Print whole Matrix
    // for(int i = 0; i < mat.dim; i++) {
    // 	for(int j = 0; j < mat.dim; j++) {

    // 	}
    // }
    return os;
  }
};
} // namespace cuda
} // namespace DPsim

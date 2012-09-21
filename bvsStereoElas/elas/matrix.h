#ifndef MATRIX_H
#define MATRIX_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
/*** added ↓ ***/
#include <limits>

// define fixed-width datatypes for Visual Studio projects
#ifndef _MSC_VER
  #include <stdint.h>
#else
  typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#endif

#define endll endl << endl // double end line definition

typedef double FLOAT;      // double precision
//typedef float  FLOAT;    // single precision

class Matrix {

public:

  // constructor / deconstructor
  Matrix ();                                                  // init empty 0x0 matrix
  Matrix (const int32_t m,const int32_t n);                   // init empty mxn matrix
  Matrix (const int32_t m,const int32_t n,const FLOAT* val_); // init mxn matrix with values from array 'val'
  Matrix (const Matrix &M);                                   // creates deepcopy of M
  ~Matrix ();

  // assignment operator, copies contents of M
  Matrix& operator= (const Matrix &M);

  Matrix reshape(int32_t icount, int32_t jcount);

  // copies submatrix of M into array 'val', default values copy whole row/column/matrix
  void getData(FLOAT* val_,int32_t i1=0,int32_t j1=0,int32_t i2=-1,int32_t j2=-1);

  // set or get submatrices of current matrix
  Matrix getMat(int32_t i1,int32_t j1,int32_t i2=-1,int32_t j2=-1);
  /*** added ↓ ***/
  Matrix getMat(const int32_t i1,const int32_t j1,const int32_t i2,const int32_t j2, const int32_t istep, const int32_t jstep);
  /*** changed ↓ ***/
  void setMat(const Matrix &M,const int32_t i1,const int32_t j1);
  /*** added ↓ ***/
  void setMat(const Matrix &M,const int32_t i1,const int32_t j1,const int32_t istep, const int32_t jstep);

  // set sub-matrix to scalar (default 0), -1 as end replaces whole row/column/matrix
  void setVal(FLOAT s,int32_t i1=0,int32_t j1=0,int32_t i2=-1,int32_t j2=-1);

  /*** changed ↓ ***/ // set (part of) diagonal to scalar, -1 as end replaces whole diagonal
  Matrix& setDiag(FLOAT s,int32_t i1=0,int32_t i2=-1);

  // clear matrix
  void zero();

   /*** added ↓ ***/ // sum columns of matrix and return a row matrix
  Matrix sumColumns();
   /*** added ↓ ***/ // sum columns of matrix and return a column matrix (at no additional cost)
  Matrix sumColumnsT();
   /*** added ↓ ***/ // sum rows of matrix and return a column matrix
  Matrix sumRows();
   /*** added ↓ ***/ // sum rows of matrix and return a row matrix (at no additional cost)
  Matrix sumRowsT();

  // extract columns with given index
  Matrix extractCols (std::vector<int> idx);

  // create identity matrix
  static Matrix eye (const int32_t m);
  void          eye ();

  // create diagonal matrix with nx1 or 1xn matrix M as elements
  static Matrix diag(const Matrix &M);
  
  // returns the m-by-n matrix whose elements are taken column-wise from M
  static Matrix reshape(const Matrix &M,int32_t m,int32_t n);

  // create 3x3 rotation matrices (convention: http://en.wikipedia.org/wiki/Rotation_matrix)
  static Matrix rotMatX(const FLOAT &angle);
  static Matrix rotMatY(const FLOAT &angle);
  static Matrix rotMatZ(const FLOAT &angle);

  // simple arithmetic operations
  Matrix  operator+ (const Matrix &M); // add matrix
  Matrix  operator- (const Matrix &M); // subtract matrix
  Matrix  operator* (const Matrix &M); // multiply with matrix
  Matrix  operator* (const FLOAT &s);  // multiply with scalar
  Matrix  operator/ (const Matrix &M); // divide elementwise by matrix (or vector)
  Matrix  operator/ (const FLOAT &s);  // divide by scalar
  Matrix  operator- ();                // negative matrix
  Matrix  operator~ ();                // transpose
  /*** added ↓ ***/
  Matrix arrayPow(const FLOAT &exp);   // pow each matrix element with exp
  /*** added ↓ ***/
  Matrix arrayPow(const Matrix &P);    // pow each matrix element with the corresponding of P Matrix
  FLOAT   l2norm ();                   // euclidean norm (vectors) / frobenius norm (matrices)
  FLOAT   mean ();                     // mean of all elements in matrix

  // complex arithmetic operations
  static Matrix cross (const Matrix &a, const Matrix &b);    // cross product of two vectors
  static Matrix inv (const Matrix &M);                       // invert matrix M
  bool   inv ();                                             // invert this matrix
  FLOAT  det ();                                             // returns determinant of matrix
  bool   solve (const Matrix &M,FLOAT eps=1e-20);            // solve linear system M*x=B, replaces *this and M
  bool   lu(int32_t *idx, FLOAT &d, FLOAT eps=1e-20);        // replace *this by lower upper decomposition
  void   svd(Matrix &U,Matrix &W,Matrix &V);                 // singular value decomposition *this = U*diag(W)*V^T

  // print matrix to stream
  friend std::ostream& operator<< (std::ostream& out,const Matrix& M);

  // direct data access
  FLOAT   **val;
  int32_t   m,n;

private:

  void allocateMemory (const int32_t m_,const int32_t n_);
  void releaseMemory ();
  inline FLOAT pythag(FLOAT a,FLOAT b);

};

#endif // MATRIX_H

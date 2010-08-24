/**
 * @file Matrix.h
 * @brief Basic dense matrix library.
 * @author Michael Kaess
 * @version $Id: Matrix.h 2736 2010-08-04 20:24:05Z kaess $
 *
 * Copyright (C) 2009-2010 Massachusetts Institute of Technology.
 * Michael Kaess (kaess@mit.edu) and John J. Leonard (jleonard@mit.edu)
 *
 * This file is part of iSAM.
 *
 * iSAM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * iSAM is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with iSAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include <iostream>

namespace isam {

class Matrix {
protected:
  double* _data;
  int _num_rows, _num_cols;

private:
  void _init(int r, int c);

public:
  Matrix();
  Matrix(const Matrix& rhs);
  Matrix(int r, int c, const double * const mat = NULL);
  Matrix(int r0, int c0, int dr, int dc, const Matrix& mat);
  ~Matrix();

  const double* data() const;
  void print() const;

  double& operator()(int r, int c);
  const double& operator()(int r, int c) const;

  const Matrix& operator= (const Matrix& rhs);

  // collect
  const Matrix operator| (const Matrix& rhs) const;

  // stack
  const Matrix operator^(const Matrix& rhs) const;

  const Matrix operator+(const Matrix& rhs) const;
  const Matrix operator-(const Matrix& rhs) const;
  const Matrix operator-() const;
  const Matrix operator*(const Matrix& rhs) const;

  const double get(int r, int c) const;
  const void set(int r, int c, double val);

  const Matrix transpose() const;

  const double trace() const;

  inline int num_rows() const {return _num_rows;}
  inline int num_cols() const {return _num_cols;}

  static Matrix zeros(int r, int c);
  static Matrix zeros(int n);
  static Matrix eye(int n);
  // n-th unit vector with r rows
  static Matrix unit(int r, int nth);

  friend class Vector;
};

Matrix make_Matrix(int r, int c, ...); // make sure to explicitly specify floats ("1.0" instead of "1")

void collect(const Matrix &A, const Matrix &B, Matrix &C);
void stack(const Matrix &A, const Matrix &B, Matrix &C);
std::ostream& operator<< (std::ostream& s, const Matrix &M);
const Matrix operator*(double lhs, const Matrix& rhs);
const Matrix operator/(const Matrix& lhs, double rhs);

}

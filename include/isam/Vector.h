/**
 * @file Vector.h
 * @brief Basic dense vector library.
 * @author Michael Kaess
 * @version $Id: Vector.h 2782 2010-08-16 18:44:12Z kaess $
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

#include "Matrix.h"

namespace isam {

class Vector : public Matrix {
public:
  Vector();
  Vector(const Matrix& rhs);

  /**
   * Copy constructor.
   * @param rhs Matrix to copy from.
   */
  Vector(const Vector& rhs);

  /**
   * Constructor based on double array.
   * @param r Number of entries.
   * @param mat Double array with vector values.
   */
  Vector(int r, const double * const mat = NULL);

  /**
   * Create sub-vector by constructor.
   * @param first First entry to copy.
   * @param num Number of entries to copy.
   * @param vec Vector to copy from.
   */
  Vector(int first, int num, const Vector& vec);
  ~Vector();

  const int size() const;

  const Vector& operator= (const Vector& rhs);
  double& operator()(int r);
  const double& operator()(int r) const;

  const Vector operator+(const Vector& rhs) const {return (Vector)Matrix::operator+(rhs);}
  const Vector operator-(const Vector& rhs) const {return (Vector)Matrix::operator-(rhs);}
  const Vector operator-() const {return (Vector)Matrix::operator-();}
  const Vector operator*(const Vector& rhs) const;

  double dot(const Vector& rhs) const;

  const Matrix diag_matrix() const;

  const double get(int r);
  const void set(int r, double val);

  const Vector abs() const;
  double max() const;
  double norm_inf() const;
  double norm2() const;
  double norm() const;

  /**
   * Insert new elements into the vector, initialized with 0.
   * @param num Number of new entries.
   * @param pos Position to insert: 0 means front, -1 or num_rows means at the end.
   */
  void add_new_rows(int num, int pos = -1);
};

Vector make_Vector(int r, ...); // make sure to explicitly specify floats ("1.0" instead of "1")

const Vector operator*(const Matrix& lhs, const Vector& rhs);
const Vector operator/(const Vector& lhs, double rhs);
const Vector operator*(double lhs, const Vector& rhs);

}

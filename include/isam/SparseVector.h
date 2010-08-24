/**
 * @file SparseVector.cpp
 * @brief part of sparse matrix functionality for iSAM
 * @author Michael Kaess
 * @version $Id: SparseVector.h 2782 2010-08-16 18:44:12Z kaess $
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

namespace isam {

class SparseVector {
  int _nnz;
  int _nnz_max;
  int* _indices;
  double* _values;

  /**
  * Copy data from one sparse vector to a new one - private
  * @param vec Existing sparse vector to copy from.
  */
  void _copy_from(const SparseVector& vec);

  /**
  * Deallocate memory - private.
  */
  void _dealloc();

  /**
   * Search for an index - private;
   * @return Index of entry, or if not in list, index of preceding entry + 1.
   */
  int _search(int idx) const;

  /**
   * Resize the sparse vector - dangerous, only used internally - private.
   * @param new_nnz_max New size of vector.
   */
  void _resize(int new_nnz_max);

public:
  /**
   * Standard constructor.
   */
  SparseVector();

  /**
   * Copy constructor - note that overwriting operator= is also necessary!
   * @param vec SparseVector to initialize from.
   */
  SparseVector(const SparseVector& vec);

  /**
   * Subvector copy constructor.
   * @param vec SparseVector to copy from.
   * @param num Number of entries to copy (note sparse - most will not exist!)
   * @param first Index of first entry of new vector.
   */
  SparseVector(const SparseVector& vec, int num, int first = 0);

  /**
   * Construct from raw data.
   * @param indices Array of row indices.
   * @param values Array of values.
   * @param nnz Length of vector.
   */
  SparseVector(int* indices, double* values, int nnz);

  /**
   * Destructor.
   */
  virtual ~SparseVector();

  /**
   * Assignment operator (see also copy constructor).
   * @param vec Right-hand-side vector in assignment
   * @return self.
   */
  const SparseVector& operator= (const SparseVector& vec);

  /**
   * Access value of an entry.
   * @param idx Index of entry to access.
   * @return Value of entry.
   */
  double operator()(int idx) const;

  /**
   * Copy raw data, useful for converting to Matlab format.
   * @param indices Destination for row indices.
   * @param values Destination for values.
   */
  void copy_raw(int* indices, double* values) const;

  /**
   * Create a new entry at the end of the sparse vector.
   * Used for efficient incremental creation of SparseVector in apply_givens().
   * @param idx Index of entry to add, has to be greater than last_idx().
   * @param val Value of new entry.
   */
  void append(int idx, const double val = 0.);

  /**
   * Assign a value to a specific entry.
   * @param idx Index of entry to assign to.
   * @param val Value to assing.
   * @return True if new entry had to be created (needed to update row index)
   */
  bool set(int idx, const double val = 0.);

  /**
   * Remove an entry; simply ignores non-existing entries.
   * @param idx Index of entry to remove.
   */
  void remove(int idx);

  /**
   * Find index of first non-zero entry.
   * @return Index of first non-zero entry, or -1 if vector is empty.
   */
  int first() const;

  /**
   * Find index of last non-zero entry
   * @return Index of last non-zero entry, or -1 if vector is empty.
   */
  int last() const;

  /**
   * Shift indices starting at pos by offset.
   * @param num Number of new entries.
   * @param pos First index that should be shifted.
   */
  void add_entries(int num, int pos);

  /**
   * Prints contents of vector as a list of tuples.
   */
  void print() const;

  inline int nnz() const {
    return _nnz;
  }

  friend class SparseVectorIter;
};

class SparseVectorIter {
  const SparseVector& s;
  int index;
public:

  /**
   * Iterator for SparseVector.
   * @param sv SparseVector.
   */
  SparseVectorIter(const SparseVector& sv);

  /**
   * Check if current element valid, ie. if we have not reached the end yet.
   * @return True if we still have a valid entry.
   */
  bool valid() const;

  /**
   * Get current element index.
   * @return Current element index.
   */
  int get() const;

  /**
   * Get current element index and value.
   * @param val Current element value returned.
   * @return Current element index.
   */
  int get(double& val) const;

  /**
   * Get current element value.
   * @return Current element value returned.
   */
  double get_val() const;

  /**
   * Go to next element.
   */
  void next();
};

}

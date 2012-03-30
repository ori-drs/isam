/**
 * @file covariance.h
 * @brief Recovery of marginal covariance matrix.
 * @author Michael Kaess
 * @version $Id: covariance.h 4688 2011-06-08 13:26:07Z hordurj $
 *
 * Copyright (C) 2009-2012 Massachusetts Institute of Technology.
 * Michael Kaess, Hordur Johannsson, David Rosen and John J. Leonard
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

#include <vector>
#include <utility> // pair
#include <list>
#include <Eigen/Dense>

#include "SparseMatrix.h"

namespace isam {

typedef std::vector< std::vector<int> > index_lists_t;

typedef std::vector< std::pair<int, int> > entry_list_t;

/**
 * Takes a list of variable indices, and returns the marginal covariance matrix.
 * @param R Sparse factor matrix.
 * @param index_lists List of lists of indices; a block will be recovered for each list.
 * @param debug Optional parameter to print timing information.
 * @param step Optional parameter to print statistics (default=-1, no stats printed).
 * @return List of dense marginal covariance matrices.
 */
std::list<Eigen::MatrixXd> cov_marginal(const SparseMatrix& R,
                                        const index_lists_t& index_lists,
                                        bool debug=false, int step=-1);

/**
 * Takes a list of pairs of integers and returns the corresonding
 * entries of the covariance matrix.
 * @param R Sparse factor matrix.
 * @param entry_lists List of pairs of integers refering to covariance matrix entries.
 * @param debug Optional parameter to print timing information.
 * @param step Optional parameter to print statistics (default=-1, no stats printed).
 * @return List of doubles corresponding to the requested covariance matrix entries.
 */
std::list<double> cov_marginal(const SparseMatrix& R,
                               const entry_list_t& entry_list);

}

/**
 * @file Comparison.h
 * @brief iSAM comparison in doxygen format
 * @author Michael Kaess
 * @version $Id: Comparison.h 2924 2010-08-27 15:47:17Z kaess $
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

/** @page Comparison Why Use iSAM?

The SLAM literature often focuses on batch processing of recorded
datasets. However, for most real robotics applications the estimates
need to be available online to be useful for navigation, planning or
manipulation. iSAM focuses on online operation by providing the best
estimate available at any time while keeping the computational
requirements low.

iSAM has successfully been used online on a range of mobile robot
platforms, including ground robots (DARPA LAGR platform, Willow Garage
PR2), aerial robots (quadrotors), and underwater robots (Bluefin HAUV,
Hydroid REMUS 100).

@section why_comparison Comparison

- Fastest of the state-of-the-art SLAM algorithms
- Provides the exact least-squares solution
- Provides efficient access to marginal covariances
- Also deals with landmarks, not just pose graphs
- Easy to generalize to new cost functions

\image html comparison_timing.png

We use Square Root SAM as a basis for comparison, as it provides the
exact least-squares solution. iSAM also provides the exact
least-square solution, but with the caveat that the most recently
added variables might not be converged yet, as relinearization is
postponed to the next periodic batch step. Sparse Pose Adjustment
(SPA2d) by Konolige et al. provides a faster implementation of square
root SAM, but batch methods in general have to solve a problem of
continously growing size, while iSAM avoids unncessary repetition of
calculations. HOG-Man by Grisetti et al. also reduces unncessary
computations using a hierarchical approach, but only provides an
approximate solution. Note that HOG-Man can provide the exact
solution, but will then be slower again and essentially perform square
root SAM.

We have omitted other state-of-the-art algorithms such as TORO and
Treemap, as they have already been shown to be inferior to SPA2d. In
particular, TORO does not correctly deal with angular measurements,
which is especially problematic when used in 3D. In contrast, Square
Root SAM and iSAM have already been shown to work in 3D vision
applications in IJRR 2006 and my thesis work.

Note that iSAM can be slow on some simulated datasets, however, we
have not encountered such data in real applications yet. Also, our
latest research has lead us to an improved iSAM algorithm based on the
Bayes tree that resolves this problem. The slowdown is caused by local
fill-in between periodic variable reordering steps. The fill-in is
caused by large numbers of loop closing constraints in certain
simulated test datasets. However, including such a large number of
loop closing constraints creates an artificial problem while not
improving the map quality. Note that standard datasets such as Intel,
Killian Court and Victoria Park have sparse loop closing constraints,
yielding good maps without any slowdown.


@section why_others Other SLAM Software

Good sources for other SLAM implementations and datasets are:

- <a href="http://www.openslam.org" target="_top">OpenSLAM.org</a>

- <a href="http://radish.sourceforge.net/" target="_top">Radish</a>

- <a href="http://www.ros.org/" target="_top">ROS</a>


@section why_references References

@li "Square Root SAM: Simultaneous Localization and Mapping via Square
Root Information Smoothing" by F. Dellaert and M. Kaess, Intl. J. of
Robotics Research, IJRR, vol. 25, no. 12, Dec. 2006, pp. 1181-1204, <a
href="http://www.cc.gatech.edu/~kaess/pub/Dellaert06ijrr.pdf"
target="_top">PDF</a>

@li "iSAM: Incremental Smoothing and Mapping" by M. Kaess,
A. Ranganathan, and F. Dellaert, IEEE Trans. on Robotics, TRO,
vol. 24, no. 6, Dec. 2008, pp. 1365-1378, <a
href="http://www.cc.gatech.edu/~kaess/pub/Kaess08tro.pdf"
target="_top">PDF</a>

@li "The Bayes Tree: Enabling Incremental Reordering and Fluid
Relinearization for Online Mapping" by M. Kaess, V. Ila, R. Robers and
F. Dellaert, Computer Science and Artificial Intelligence Laboratory,
MIT technical report MIT-CSAIL-TR-2010-021, Jan. 2010, <a
href="http://www.cc.gatech.edu/~kaess/pub/Kaess10tr.pdf"
target="_top">PDF</a>

@li "Incremental Smoothing and Mapping" by M. Kaess,
Ph.D. dissertation, Georgia Institute of Technology, Dec. 2008, <a
href="http://www.cc.gatech.edu/~kaess/pub/Kaess08thesis.pdf"
target="_top">PDF</a>

@li "Hierarchical Optimization on Manifolds for Online 2D and 3D
Mapping" by G. Grisetti, R. Kuemmerle, C. Stachniss, U. Frese, and
C. Hertzberg, IEEE Intl. Conf. on Robotics and Automation (ICRA), May
2010

@li "Sparse Pose Adjustment for 2D Mapping" by K. Konolige,
G. Grisetti, R. Kuemmerle, W. Burgard, L. Benson, R. Vincent, IEEE/RSJ
Intl. Conf. on Intelligent Robots and Systems (IROS), Oct 2010

*/

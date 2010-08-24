/**
 * @file Viewer.h
 * @brief 3D visualization.
 * @author Michael Kaess
 * @version $Id: Viewer.h 2872 2010-08-21 21:11:59Z kaess $
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

#include <vector>
#include <string>

#include "Collections.h"

class Viewer {

public:

  /**
   * GLUT initialization, open window.
   */
  void init(int (*process)(void*));

  /**
   *
   */
  void set_nodes(const std::vector<isam::Pose3d>& nodes,
      int id, const std::string& name, int type);

  /**
   *
   */
  void set_links(const std::vector<std::pair<int,int> >& links,
      int id, const std::string& name, int collection1, int collection2);

};

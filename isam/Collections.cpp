/**
 * @file Collections.cpp
 * @brief 3D visualization.
 * @author Michael Kaess
 * @version $Id: Collections.cpp 2885 2010-08-23 03:53:45Z kaess $
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

#include <vector>
#include <map>

#if defined(__APPLE__) && defined(__MACH__)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include "isam/Pose3d.h"

#include "Collections.h"

using namespace std;
using namespace isam;

#define rad_to_degrees(rad) ((rad)*180/M_PI)

float colors[] = {
  1.0, 0.0, 0.0, // red
  0.0, 1.0, 0.0, // green
  0.0, 0.0, 1.0, // blue
  1.0, 1.0, 0.0,
  1.0, 0.0, 1.0,
  0.0, 1.0, 1.0,
  0.5, 1.0, 0.0,
  1.0, 0.5, 0.0,
  0.5, 0.0, 1.0,
  1.0, 0.0, 0.5,
  0.0, 0.5, 1.0,
  0.0, 1.0, 0.5,
	1.0, 0.5, 0.5, 
	0.5, 1.0, 0.5, 
	0.5, 0.5, 1.0, 
	0.5, 0.5, 1.0,
	0.5, 1.0, 0.5,
	0.5, 0.5, 1.0
};
const int num_colors = sizeof(colors)/(3*sizeof(float));

collections_t collections;

GLUquadricObj* quadric_internal = NULL;

GLUquadricObj* getQuadric() {
  if (quadric_internal==NULL) {
    quadric_internal = gluNewQuadric();
  }
  return quadric_internal;
}

void draw_tree(double x, double y, double z) {
  glPushAttrib(GL_CURRENT_BIT);
  glPushMatrix();
  glTranslatef(x, y, z);
  glColor3f(139./255., 69./255., 19./255.);
  glPushMatrix();
  glTranslatef(0,0,0.01);
  gluCylinder(getQuadric(), 0.2, 0.2, 2.6, 8, 8);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(0,0,4.5);
  glColor3f(0.1,0.7,0.1);
  gluSphere(getQuadric(), 2., 8, 8);
  glPopMatrix();
  glPopMatrix();
  glPopAttrib();
}

void draw_tetra(double x, double y, double z, double yaw, double pitch, double roll, double size, bool mark) {
  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(rad_to_degrees(yaw),  0., 0., 1.);
  glRotatef(rad_to_degrees(pitch),0., 1., 0.);
  glRotatef(rad_to_degrees(roll), 1., 0., 0.);

  if (mark) {
	  glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    gluSphere(getQuadric(), size*1.5, 5, 5);
	  glPopAttrib();
  }
  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  // draw outline in black
  glPushAttrib(GL_CURRENT_BIT);
  glColor3f(0,0,0);
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glPopAttrib();
  glPopMatrix();
}

ObjCollection::ObjCollection(int id, string name, int type, const vector<Pose3d>& nodes) : Collection(id, name, type) {
  maxid = nodes.size()-1;
  int i = 0;
  for (vector<Pose3d>::const_iterator it = nodes.begin(); it!=nodes.end(); it++, i++) {
    objs.insert(make_pair(i, *it));
  }
}

void ObjCollection::draw() {
  glPushAttrib(GL_CURRENT_BIT);
  glColor3fv(&colors[3*(id%num_colors)]);
  // preparations
  switch(type) {
  case VIEWER_OBJ_TREE:
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_RESCALE_NORMAL);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    break;
  }
  // draw each object
  for (objs_t::const_iterator it = objs.begin(); it != objs.end(); it++) {
    int obj_id = it->first;
    const Pose3d& obj = it->second;
    // only draw if within range
    double size = 0.1; // size of plotted 3D poses
    bool is_last = (maxid == obj_id);
    switch(type) {
    case VIEWER_OBJ_POSE3D:
      draw_tetra(obj.x(), obj.y(), obj.z(), obj.yaw(), obj.pitch(), obj.roll(), size, is_last);
      break;
    case VIEWER_OBJ_TREE:
      draw_tree(obj.x(), obj.y(), obj.z());
      break;
    }
  }
  // cleanup
  switch(type) {
  case VIEWER_OBJ_TREE:
    glPopAttrib();
    break;
  }
  glPopAttrib();
}

LinkCollection::LinkCollection(int id, string name, const vector<pair<int,int> >& links, int col1, int col2)
  : Collection(id, name, type), elements(links), col1(col1), col2(col2) {}

void LinkCollection::draw() {
  glPushAttrib(GL_CURRENT_BIT);
  glColor3fv(&colors[3*(id%num_colors)]);
  int i=0;
  for (elements_t::iterator it = elements.begin(); it != elements.end(); it++, i++) {
    pair<int, int>& link = *it;
    collections_t::iterator collection_it1 = collections.find(col1);
    collections_t::iterator collection_it2 = collections.find(col2);
    if (collection_it1 != collections.end()
        && collection_it2 != collections.end()) {
      const ObjCollection::objs_t& objs1 = ((ObjCollection*)collection_it1->second)->objs;
      ObjCollection::objs_t::const_iterator it1 = objs1.find(link.first);
      const ObjCollection::objs_t& objs2 = ((ObjCollection*)collection_it2->second)->objs;
      ObjCollection::objs_t::const_iterator it2 = objs2.find(link.second);
      if (it1 != objs1.end() && it2 != objs2.end()) {
        const Pose3d& obj1 = it1->second;
        const Pose3d& obj2 = it2->second;
        glBegin(GL_LINES);
        glVertex3f(obj1.x(), obj1.y(), obj1.z());
        glVertex3f(obj2.x(), obj2.y(), obj2.z());
        glEnd();
      }
    }
  }
  glPopAttrib();
}

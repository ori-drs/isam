/**
 * @file Viewer.cpp
 * @brief 3D visualization.
 * @author Michael Kaess
 * @version $Id: Viewer.cpp 2901 2010-08-24 04:53:01Z kaess $
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
#include "SDL.h"
#include "SDL_thread.h"

#include "isam/isam.h"

#include "Collections.h"
#include "Viewer.h"

using namespace std;
using namespace isam;

// current window size, with initial size given
GLsizei width = 800;
GLsizei height = 600;

Pose3d eye;

GLuint gl_list;

SDL_Thread* thread;
SDL_mutex* mutex;

// set to true whenever new drawing primitives were added
bool redisplay_requested = false;

void populateGLList() {
  glNewList(gl_list, GL_COMPILE_AND_EXECUTE);
  for (collections_t::iterator it = collections.begin(); it != collections.end(); it++) {
    it->second->draw();
  }
  glEndList();
}

void drawGL() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  /* Camera rotation */
  glTranslated(-eye.x(), -eye.y(), -eye.z());
  glRotated(eye.roll(),  1.0f, 0.0f, 0.0f);
  glRotated(eye.pitch(), 0.0f, 1.0f, 0.0f);
  glRotated(eye.yaw(),   0.0f, 0.0f, 1.0f);

  GLfloat light_position0[] = {1.0, 1.0, 1.0, 0.0};
  glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

  // redraw (re-populate the GL list) only if something changed
  SDL_LockMutex(mutex);
  if (redisplay_requested) {
    redisplay_requested = false;
    populateGLList();
  }
  SDL_UnlockMutex(mutex);

  // draws the list
  glCallList(gl_list);
}

void initGL() {
  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_COLOR_MATERIAL);
  //  glEnable(GL_CULL_FACE);
}

void quit() {
  SDL_QuitSubSystem(SDL_INIT_VIDEO);
  SDL_WaitThread(thread, NULL);
  SDL_DestroyMutex(mutex);
  SDL_Quit();
}

void resize(GLsizei w, GLsizei h) {
  if (w<10) {
    w = 10;
  }
  if (h<10) {
    h = 10;
  }
  width = w;
  height = h;
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, (float)w/(float)h, 0.1, 1000.0);
  glMatrixMode(GL_MODELVIEW);
}

// translation and scaling is more sensitive if further away from
// origin to allow for fast navigation
double factor() {
  double distance = eye.trans().vector().norm();
  return 1. + sqrt(distance) / 2.;
}

void rotate(int dx, int dy) {
  eye.set_roll (eye.roll()  + dy);
  eye.set_pitch(eye.pitch() + dx);
}

void translate(int dx, int dy) {
  eye.set_x(eye.x() - dx * 0.03*factor());
  eye.set_y(eye.y() + dy * 0.03*factor());
}

void scale(int dx, int dy) {
  eye.set_z(eye.z() + dy * 0.1*factor());
}

void reset() {
  eye = Pose3d(0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f);
}

void keyPress(SDL_keysym *keysym) {
  switch(keysym->sym) {
  case SDLK_ESCAPE:
  case SDLK_q:
    quit();
    break;
  case SDLK_r:
    reset();
    break;
  default:
    break;
  }
}

void processMouseWheel(int button) {
  if(button == SDL_BUTTON_WHEELUP) {
    scale(0, -2.*factor());
  } else if(button == SDL_BUTTON_WHEELDOWN) {
    scale(0, 2.*factor());
  }
}

void mouseMotion(int x, int y) {
  static int x_pos = 0;
  static int y_pos = 0;

  // ignore if mouse outside window - avoids strange behavior on Mac
  if (x==0 || y==0 || x==width-1 || y==height-1) return;

  int dx = x - x_pos;
  int dy = y - y_pos;
  SDLMod modState = SDL_GetModState();
  Uint8 mouseState = SDL_GetMouseState(NULL, NULL);

  if (mouseState & SDL_BUTTON(SDL_BUTTON_LEFT)) {
    if (modState & KMOD_SHIFT) {
      scale(dx, dy);
    } else if (modState & KMOD_CTRL) {
      translate(dx, dy);
    } else {
      rotate(dx, dy);
    }
  } else if (mouseState & SDL_BUTTON(SDL_BUTTON_MIDDLE)) {
    translate(dx, dy);
  } else if (mouseState & SDL_BUTTON(SDL_BUTTON_RIGHT)) {
    scale(dx, dy);
  }

  x_pos = x;
  y_pos = y;
}

int setupSDL() {
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    require(false, "Could not initialize SDL");
  }

  atexit(quit);

  // create a GL window
  int videoFlags = SDL_OPENGL|SDL_RESIZABLE|SDL_DOUBLEBUF;
  const SDL_VideoInfo* videoInfo = SDL_GetVideoInfo();
  if (videoInfo->hw_available) {
    videoFlags |= SDL_HWSURFACE;
  } else {
    videoFlags |= SDL_SWSURFACE;
  }
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  SDL_WM_SetCaption("iSAM Viewer", "opengl");

  return videoFlags;
}

void initSDL(int w, int h, int videoFlags) {
  if (SDL_SetVideoMode(w, h, 0, videoFlags) == NULL) {
    require(false, "Could not open GL window");
  }
}

int main_loop(void* unused) {
  int videoFlags = setupSDL();
  initSDL(width, height, videoFlags);
  resize(width, height);
  initGL();

  // allocate space for an OpenGL list
  gl_list = glGenLists(1);

  // SDL even loop
  bool done = false;
  bool isActive = true;
  while(!done) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
      case SDL_VIDEORESIZE:
        initSDL(event.resize.w, event.resize.h, videoFlags);
        resize(event.resize.w, event.resize.h);
        initGL();
        break;
      case SDL_KEYDOWN:
        keyPress(&event.key.keysym);
        break;
      case SDL_MOUSEBUTTONDOWN:
        processMouseWheel(event.button.button);
        break;
      case SDL_MOUSEMOTION:
        mouseMotion(event.button.x, event.button.y);
        break;
      case SDL_ACTIVEEVENT:
        if ((event.active.gain==0) && (event.active.state & SDL_APPACTIVE)) {
          isActive = false;
        } else {
          isActive = true;
        }
        break;
      case SDL_QUIT:
        done = true;
        break;
      }
    }

    if (isActive) {
      drawGL();
      SDL_GL_SwapBuffers();
    }

    SDL_Delay(20); // don't eat up all CPU cycles, restrict to 50Hz

  }

  quit();

  return NULL;
}

void Viewer::init(int (*process)(void*)) {
  reset();

  mutex = SDL_CreateMutex();

  thread = SDL_CreateThread(process, NULL);
  main_loop(NULL);
}

void Viewer::set_nodes(const vector<Pose3d>& nodes, int id, const string& name, int type) {
  SDL_LockMutex(mutex);
  ObjCollection* collection = new ObjCollection(id, name, type, nodes);
  collections_t::iterator it = collections.find(id);
  if (it!=collections.end()) {
    // if previously created, deallocate and remove
    Collection* c = it->second;
    collections.erase(id);
    delete c;
  }
  collections.insert(make_pair(id, collection));
  redisplay_requested = true;
  SDL_UnlockMutex(mutex);
}

void Viewer::set_links(const vector<pair<int,int> >& links, int id, const string& name, int col1, int col2) {
  SDL_LockMutex(mutex);
  LinkCollection* collection = new LinkCollection(id, name, links, col1, col2);
  collections_t::iterator it = collections.find(id);
  if (it!=collections.end()) {
    // if previously created, deallocate and remove
    Collection* c = it->second;
    collections.erase(id);
    delete c;
  }
  collections.insert(make_pair(id, collection));
  redisplay_requested = true;
  SDL_UnlockMutex(mutex);
}

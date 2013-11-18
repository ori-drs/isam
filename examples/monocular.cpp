/**
 * @file monocular.cpp
 * @brief Example of using monocular factors.
 * @author Michael Kaess
 * @version $Id: monocular.cpp 9315 2013-11-18 20:23:34Z kaess $
 */

#include <iostream>
#include <iomanip>

#include <isam/isam.h>
#include <isam/slam_monocular.h>
#include <isam/robust.h>

using namespace std;
using namespace isam;
using namespace Eigen;

// monocular camera parameters
const double f = 360;  // focal length in pixels
const double u0 = 240; // principal point in pixels
const double v0 = 120;

double robust_cost_function(double d) {
  return cost_pseudo_huber(d, .5);
}

void simple_monocular() {

  Vector2d pp(u0, v0);
  MonocularCamera camera(f, pp);

  Pose3d origin;
  Point3d p0(5.,1.,2.);

  Slam slam;

  // first monocular camera
  Pose3d_Node* pose0 = new Pose3d_Node();
  slam.add_node(pose0);

  // create a prior on the camera position
  Noise noise6 = Information(100. * eye(6));
  Pose3d_Factor* prior = new Pose3d_Factor(pose0, origin, noise6);
  slam.add_factor(prior);

  // second monocular camera
  Pose3d_Node* pose1 = new Pose3d_Node();
  slam.add_node(pose1);

  // add odometry measurement
  Pose3d delta(1.,0,0, 0,0,0); // move one meter forward (noise-free measurement...)
  Pose3d_Pose3d_Factor* odo = new Pose3d_Pose3d_Factor(pose0, pose1, delta, noise6);
  slam.add_factor(odo);

  // add some monocular measurements
  Point3d_Node* point = new Point3d_Node();
  slam.add_node(point);
  Noise noise3 = Information(eye(3));
  // first monocular camera projection
  MonocularMeasurement measurement0 = camera.project(origin, p0);
  cout << "Projection in first camera:" << endl;
  cout << measurement0 << endl;
  measurement0.u += 0.5; // add some "noise"
  measurement0.v -= 0.2;
  Monocular_Factor* factor1 = new Monocular_Factor(pose0, point, &camera, measurement0, noise3);
  slam.add_factor(factor1);
  // second monocular camera projection
  MonocularMeasurement measurement1 = camera.project(delta, p0);
  measurement1.u -= 0.3; // add some "noise"
  measurement1.v += 0.7;
  Monocular_Factor* factor2 = new Monocular_Factor(pose1, point, &camera, measurement1, noise3);
  slam.add_factor(factor2);

  cout << "Before optimization:" << endl;
  cout << setprecision(3) << setiosflags(ios::fixed);
  cout << point->value() << endl;
  cout << pose0->value() << endl;
  cout << pose1->value() << endl;

  // not needed here, but for more complex examples use Powell's dog leg and
  // optionally a robust estimator (pseudo Huber) to deal with occasional outliers
  Properties prop = slam.properties();
  prop.method = DOG_LEG;
  slam.set_properties(prop);
//  slam.set_cost_function(robust_cost_function);

  // optimize
  slam.batch_optimization();

  cout << "After optimization:" << endl;
  cout << point->value() << endl;
  cout << pose0->value() << endl;
  cout << pose1->value() << endl;
}

int main() {

  simple_monocular();

  return 0;
}

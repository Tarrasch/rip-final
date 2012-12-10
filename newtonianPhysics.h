#ifndef __NEWTONIAN_PHYSICS_H_
#define __NEWTONIAN_PHYSICS_H_

#include <iostream>
#include <stdlib.h>
#include <list>
#include <Eigen/LU>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;
typedef vector<VectorXd> Path;

const double g = (-9.82/10.0);
const double pg = -g;
const double PI = 3.1415925;
const double dt = 0.05;
const double theta = 45.0;

Path projectileMotion(VectorXd pos, VectorXd vel, VectorXd acc);
VectorXd calculateVelocities(VectorXd start, VectorXd end);
Path straightMotion(VectorXd start, VectorXd endpos);

Path projectileMotionWRandT(VectorXd pos, VectorXd vel, VectorXd acc, int randMaxAcc, double maxTime);
Path projectileMotionWRand(VectorXd pos, VectorXd vel, VectorXd acc, int randMaxAcc);
#endif

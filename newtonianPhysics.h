#ifndef __NEWTONIAN_PHYSICS_H_
#define __NEWTONIAN_PHYSICS_H_

#include <iostream>
#include <stdlib.h>
#include <list>
#include <Eigen/LU>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

const double g = (-9.82/10.0);
const double pg = -g;
const double PI = 3.1415925;
const double dt = 0.05;
const double theta = 45.0;

std::list< VectorXd > projectileMotion(VectorXd pos, VectorXd vel, VectorXd acc);
VectorXd calculateVelocities(VectorXd start, VectorXd end);
list< VectorXd > straightMotion(VectorXd start, VectorXd endpos);

std::list< VectorXd > projectileMotionWRandT(VectorXd pos, VectorXd vel, VectorXd acc, int randMaxAcc, double maxTime);
std::list< VectorXd > projectileMotionWRand(VectorXd pos, VectorXd vel, VectorXd acc, int randMaxAcc);
#endif

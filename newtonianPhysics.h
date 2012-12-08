#ifndef __NEWTONIAN_PHYSICS_H_
#define __NEWTONIAN_PHYSICS_H_

#include <iostream>
#include <stdlib.h>
#include <list>
#include <Eigen/LU>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

std::list< VectorXd > projectileMotion(VectorXd pos, VectorXd vel, VectorXd acc);
VectorXd calculateVelocities(VectorXd start, VectorXd end);
list< VectorXd > straightMotion(VectorXd start, VectorXd endpos);
#endif

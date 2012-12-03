#ifndef __NEWTONIAN_PHYSICS_H_
#define __NEWTONIAN_PHYSICS_H_

#include <iostream>
#include <stdlib.h>
#include <list>
#include <Eigen/LU>
#include <Eigen/Core>
#define PRINT(x) std::cout << #x << " = " << x << std::endl;

using namespace std;
using namespace Eigen;

std::list< VectorXd > projectileMotion(VectorXd pos, VectorXd vel);

#endif

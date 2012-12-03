#include <iostream>
#include <stdlib.h>
#include <Eigen/LU>
#include "newtonianPhysics.h"
#define PRINT(x) std::cout << #x << " = " << x << std::endl;

std::list< VectorXd > projectileMotion(VectorXd pos, VectorXd vel){
  VectorXd acc;
  double const g = -9.82/10.0;
  acc << 0, 0, g;

  std::list<VectorXd> res;
  double dt = 0.05;

  do {
    res.push_back(pos);
    pos += vel*dt + acc*dt*dt/2.0;
    vel += acc*dt;
  } while (pos[2] > 0);

  return res;
}

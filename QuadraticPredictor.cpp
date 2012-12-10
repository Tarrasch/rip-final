#include <iostream>
#include <stdlib.h>
#include <Eigen/LU>
#include <Eigen/Core>
#include <vector>
#include <list>
#include <robotics/World.h>
#include <wx/wx.h>
#include "QuadraticPredictor.h"
#define PRINT(x) std::cout << #x << " = " << x << std::endl;
#define ECHO(x) std::cout << x << std::endl;

using namespace std;
using namespace Eigen;

Path QuadraticPredictor::getPredictedPath(Path observedPath, double time){
	// Only continue if the observed path is greater than 0
	int n = observedPath.size();
	assert(n > 0);
	const int m = min(5,(n/3));

	VectorXd vel(3); vel<<0,0,0;
	VectorXd acc(3); acc<<0,0,0;
  if(n >= 3*m){
    VectorXd b0Last = sum(observedPath.end()-m, observedPath.end())*(1.0/m);
    VectorXd b1Last = sum(observedPath.end()-(2*m), observedPath.end()-m)*(1.0/m);
    VectorXd b2Last = sum(observedPath.end()-(3*m), observedPath.end()-(2*m))*(1.0/m);
    VectorXd vel1 = (b0Last - b1Last)*(1.0/(dt*m));
    VectorXd vel2 = (b1Last - b2Last)*(1.0/(dt*m));
    VectorXd acc = (vel1 - vel2)*(1.0/dt);
    vel = vel1;
  }
	return projectileMotionWRandT(observedPath.back(), vel, acc, 0, time);
}


#include <iostream>
#include <stdlib.h>
#include <Eigen/LU>
#include <Eigen/Core>
#include <vector>
#include <list>
#include <robotics/World.h>
#include <wx/wx.h>
#include "LinearPredictor.h"
#define PRINT(x) std::cout << #x << " = " << x << std::endl;
#define ECHO(x) std::cout << x << std::endl;

using namespace std;
using namespace Eigen;

Path LinearPredictor::getPredictedPath(Path observedPath, double time){
	// Only continue if the observed path is greater than 0
	int n = observedPath.size();
	assert(n > 0);
	const int m = 5;

	VectorXd vel(3); vel<<0,0,0;
  if(n >= 2*m){
    ECHO("HII");
    VectorXd last = accumulate(observedPath.end()-m, observedPath.end(), vel)/m;
    PRINT(last);
    VectorXd beforeLast = accumulate(observedPath.end()-(2*m), observedPath.end()-m, vel)/m;
    PRINT(beforeLast);
    vel = (last - beforeLast)*(1/dt);
  }
	VectorXd acc(3); acc<<0,0,0;
	return projectileMotionWRandT(observedPath.back(), vel, acc, 0, time);
}


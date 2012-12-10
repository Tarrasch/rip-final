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

LinearPredictor::LinearPredictor(std::list<Eigen::VectorXd> observedPath, double time){
	int pathSize = observedPath.size();
	
	// Only continue if the observed path is greater than 0
	assert(pathSize > 0);
	
	// Grab the last point in the path
	VectorXd p2(3);	// = observedPath.back();
	p2 << 0,0,0;
	VectorXd p1(3); //= *it;
	p1 << 0,0,0;
	
	// Build an iterator over the path list
	list<VectorXd>::iterator it = observedPath.end();
	it--;
	
	// If there is only 1 path point (occurs at the very "start" of time.) just use that one point
	if (pathSize == 1)
	{
		p1 = *it;
	}
	else if (pathSize == 2)
	{
		it--;
		p1 = *it;
	}
	else if (pathSize == 3)
	{
		it--;
		p2 += *it;
		p2 = p2 * (1/2);
		it--;
		p1 = *it;
	}
	else
	{
		int i = pathSize;
		while (i > pathSize/2)
		{
			p2 += *it;
			it--;
			i--;
		}
		p2 = p2 * (2/pathSize);

		while (i > 0)
		{
			p1 += *it;
			it--;
			i--;
		}
		p1 = p1 * (2/pathSize);
	}

  VectorXd vel = pathSize == 1 ?
                 (p2 - p1)/dt :
                 (p2 - p1)/((pathSize-1)*dt);

	VectorXd acc(3); acc<<0,0,0;

	predictedPath = projectileMotionWRandT(p2, vel, acc, 0, time);
}

std::list<Eigen::VectorXd> LinearPredictor::getPredictedPath(){
        return predictedPath;
}

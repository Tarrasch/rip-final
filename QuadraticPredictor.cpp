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
		// Terminate if the path size is zero elements
		assert(observedPath.size() > 0);
		
		// Build an iterator over the entire path. Start at the "end" of the path. 
		//Note: this "end" node doesn't actually contain the end of the path but a special node that is 1 past the true end
		Path::iterator it = observedPath.end();
		
		// Decrement the iterator and grab the end node
		it--;
		VectorXd p3 = *it;
		
		// Check to make sure we aren't at the beginning now (ie, 1 node paths). If not decremenet the iterator
		if(it != observedPath.begin()) { 
			it--;
		}
		
		// Grab the second to end node
		VectorXd p2 = *it;
		
		// Check to make sure we aren't at the beginning now (ie, 2 node paths). If not decremenet the iterator
		if(it != observedPath.begin()) { 
			it--;
		}
		
		// Grab the third to end node
		VectorXd p1 = *it;
		
		/*Debug the three points
		PRINT(p1);
		PRINT(p2);
		PRINT(p3);*/
		
		VectorXd v1 = (p2-p1)*1/dt;
		VectorXd v2 = (p3-p2)*1/dt;
		VectorXd acc = (observedPath.size() > 2) * (v2-v1)*1/dt;
		
		
		/*PRINT(v1);
		PRINT(v2);
		PRINT(acc);*/
		
		//call projectileMotionWRandT with no randomness
		predictedPath = projectileMotionWRandT(p3, v2, acc, 0, time);
		return predictedPath;
}


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

LinearPredictor::LinearPredictor(std::list<Eigen::VectorXd> observedPath) :
  Predictor(observedPath) {
        VectorXd last = observedPath.back();
        list<VectorXd>::iterator it = observedPath.end();
        it--;
        VectorXd beforeLast = *it;
        
        PRINT(last);
        PRINT(beforeLast);
        
        //call projectileMotionWRandT with no randomness
        VectorXd vel = (last - beforeLast);
        VectorXd acc(3); acc<<0,0,0;
        predictedPath = projectileMotionWRandT(beforeLast, vel, acc, 0, 4);
}

std::list<Eigen::VectorXd> LinearPredictor::getPredictedPath(){
        return predictedPath;
}

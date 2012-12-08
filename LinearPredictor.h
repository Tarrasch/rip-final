#ifndef __THROWER_H_
#define __THROWER_H_

#include <iostream>
#include <stdlib.h>
#include <Eigen/LU>
#include <Eigen/Core>
#include <vector>
#include <list>
#include <robotics/World.h>
#include <wx/wx.h>
#include "newtonianPhysics.h"

using namespace std;
using namespace Eigen;

class LinearPredictor {

private:
    std::list<Eigen::VectorXd> predictedPath;
public:       
    LinearPredictor(std::list<Eigen::VectorXd> observedPath);
    
    std::list<Eigen::VectorXd> getPredictedPath();
};

#endif


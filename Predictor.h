#ifndef __PREDICTOR_H_
#define __PREDICTOR_H_

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

class Predictor {

public:

    virtual Path getPredictedPath(Path observedPath, double time) = 0;
    virtual ~Predictor(){};
};

#endif


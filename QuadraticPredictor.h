#ifndef __QUADRATICPREDICTOR_H_
#define __QUADRATICPREDICTOR_H_

#include <iostream>
#include <stdlib.h>
#include <Eigen/LU>
#include <Eigen/Core>
#include <vector>
#include <list>
#include <robotics/World.h>
#include <wx/wx.h>
#include "newtonianPhysics.h"
#include "Predictor.h"

using namespace std;
using namespace Eigen;

class QuadraticPredictor : public Predictor {

private:
    Path predictedPath;
public:
    virtual Path getPredictedPath(Path observedPath, double time);
    virtual ~QuadraticPredictor(){};
};

#endif


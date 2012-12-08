#ifndef __LINEARPREDICTOR_H_
#define __LINEARPREDICTOR_H_

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

class LinearPredictor : public Predictor {

private:
    std::list<Eigen::VectorXd> predictedPath;
public:
    LinearPredictor(std::list<Eigen::VectorXd> observedPath);
    virtual std::list<Eigen::VectorXd> getPredictedPath();
    virtual ~LinearPredictor();
};

#endif


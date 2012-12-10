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
    std::list<Eigen::VectorXd> predictedPath;
public:
    QuadraticPredictor(std::list<Eigen::VectorXd> observedPath, double time);
    virtual std::list<Eigen::VectorXd> getPredictedPath();
    virtual ~QuadraticPredictor(){};
};

#endif


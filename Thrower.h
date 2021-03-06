#ifndef __THROWER_H_
#define __THROWER_H_

#include <iostream>
#include <stdlib.h>
#include <Eigen/LU>
#include <Eigen/Core>
#include <vector>
#include <list>
#include <robotics/World.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h> 
#include <kinematics/BodyNode.h> 
#include <kinematics/Marker.h> 
#include <wx/wx.h>
#include "newtonianPhysics.h"
#include "Predictor.h"
#define CLOSEST_RRT false
#define MULTI_RRT true

using namespace std;
using namespace Eigen;

class Thrower {

public:
    const int mRobotId;

    /// Member variables
    robotics::World &mWorld;  
    robotics::Object &mSphereActual; 
    robotics::Object &mSpherePerceived;
    robotics::Object &mSpherePredicted;
    robotics::Object &mAimStar; 

    /// Copy of gui stuff
    wxTextCtrl *mTimeText;

    Path objectPath;
    Path perceivedPath; 
    Path aims;
    Path predictedPath;
    list< Path > predictedPaths;
    Path jointPath; 

    Thrower(robotics::World &_world, wxTextCtrl *_timeText,
        robotics::Object &_sphereActual, robotics::Object &_spherePerceived, robotics::Object &_spherePredicted, robotics::Object &_aimStar);

    void throwObject(VectorXd pos, double noise, double prediction_time, Predictor &predictor, double maxnodes, bool approach);
    double SetThrowTimeline(bool addFrames);
};
#endif


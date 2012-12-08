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

using namespace std;
using namespace Eigen;

class Thrower {

public:
    typedef list<VectorXd> Path;
    const int mRobotId;

    /// Member variables
    robotics::World &mWorld;  // The world
    robotics::Object &mSphereActual; // The object (aka the trash) we throw
    robotics::Object &mSpherePredicted; // The object (aka the trash) we throw

    /// Copy of gui stuff
    wxTextCtrl *mTimeText;

    Path objectPath; // Path of object
    Path predictedPath;
    list< Path > predictedPaths;
    Path jointPath; // Path of object

    Thrower(robotics::World &_world, wxTextCtrl *_timeText,
        robotics::Object &_sphereActual, robotics::Object &_spherePredicted);

    void throwObject(VectorXd pos);
    void SetThrowTimeline();

    VectorXd findRandomReachablePosition(VectorXd pos);
    VectorXd findRandomStartPosition(VectorXd pos);
};
#endif


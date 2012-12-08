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
    const int mRobotId;

    /// Member variables
    robotics::World &mWorld;  // The world
    robotics::Object &mObject; // The object (aka the trash) we throw

    /// Copy of gui stuff
    wxTextCtrl *mTimeText;

    std::list<Eigen::VectorXd> objectPath; // Path of object
    std::list<Eigen::VectorXd> jointPath; // Path of object

    Thrower(robotics::World &_world, robotics::Object &_object, wxTextCtrl *_timeText);

    void throwObject(VectorXd pos);
       
    void SetThrowTimeline();
    
    VectorXd findRandomReachablePosition(VectorXd pos);
    VectorXd findRandomStartPosition(VectorXd pos);
};
#endif


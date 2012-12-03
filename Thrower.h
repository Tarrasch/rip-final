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

#define PRINT(x) std::cout << #x << " = " << x << std::endl;

using namespace std;
using namespace Eigen;

class Thrower {

public:

    /// Member variables
    robotics::World &mWorld;  // The world
    robotics::Object &mObject; // The object (aka the trash) we throw

    /// Copy of gui stuff
    wxTextCtrl *mTimeText;

    std::list<Eigen::VectorXd> objectPath; // Path of object

    Thrower(robotics::World &_world, robotics::Object &_object, wxTextCtrl *_timeText);

    void throwObject(VectorXd pos, VectorXd vel);

    void SetThrowTimeline();
};
#endif


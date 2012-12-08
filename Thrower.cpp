#include <stdlib.h>
#include "Thrower.h"
#include "newtonianPhysics.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/GRIPTab.h>
#include "JointMover.h"

#define PRINT(x) std::cout << #x << " = " << x << std::endl;
#define ECHO(x) std::cout << x << std::endl;

using namespace std;
using namespace Eigen;

Thrower::Thrower(robotics::World &_world, robotics::Object &_object, wxTextCtrl *_timeText)
  : mWorld(_world), mObject(_object), mTimeText(_timeText), mRobotId(0) {
  // TODO: mRobotId shouldn't be fixed 0, rather a parameter
}

// The effect of this method is that it will fill the path value
void Thrower::throwObject(VectorXd pos, VectorXd vel) {
  objectPath = projectileMotion(pos, vel);
  PRINT(objectPath.size());
  JointMover arm(mWorld, mRobotId);
  jointPath.clear();
  VectorXd joints = mWorld.getRobot(0)->getQuickDofs();
  PRINT(joints.size());
  for( list<VectorXd>::iterator it = objectPath.begin(); it != objectPath.end(); it++ ) {
    jointPath.push_back(joints);
    joints = arm.OneStepTowardsXYZ(joints, *it);
  }
}

void Thrower::SetThrowTimeline(){
    if( objectPath.size() == 0 ) {
        cout << "--(!) Must create a nonempty plan before setting timeline (!)--" << endl;
        return;
    }

    double T;
    mTimeText->GetValue().ToDouble(&T);

    int numsteps = objectPath.size();
    double increment = T/(double)numsteps;
    cout << "-->(+) Updating Timeline - Increment: " << increment
         << " Total T: " << T << " Steps: " << numsteps << endl;

    frame->InitTimer( string("Throwing of object"),increment );

    list<VectorXd>::iterator it_j;
    for( list<VectorXd>::iterator it = objectPath.begin(),
         it_j = jointPath.begin();
         it != objectPath.end();
         it++, it_j++ ) {
        VectorXd &pos = *it;
        PRINT(pos);
        PRINT(*it_j);
        mObject.setPositionXYZ(pos[0], pos[1], pos[2]);
        mObject.update();
        mWorld.getRobot(mRobotId)->setQuickDofs( *it_j );
        mWorld.getRobot(mRobotId)->update();
        frame->AddWorld( &mWorld );
    }

}

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
  : mWorld(_world), mObject(_object), mTimeText(_timeText) {

}

// The effect of this method is that it will fill the path value
void Thrower::throwObject(VectorXd pos, VectorXd vel) {
  objectPath = projectileMotion(pos, vel);
  PRINT(objectPath.size());
  JointMover arm(mWorld, 0); // TODO: repalce 0 with mRobotId
  jointPath.clear();
  VectorXd joints(6, 0.0); // Create 6 elements with 0.0, there are 6 joints I THINK
  for( list<VectorXd>::iterator it = objectPath.begin(); it != objectPath.end(); it++ ) {
    jointPath.push_back(joints);
    joints = arm.OneStepTowardsXYZ(*it, joints);
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
        mWorld.getRobot(0)->setQuickDofs( *it_j ); // TODO: repalce 0 with mRobotId
				mWorld.getRobot(0)->update();// TODO: repalce 0 with mRobotId
        frame->AddWorld( &mWorld );
    }

}

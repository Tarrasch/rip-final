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
#define ARM_LENGTH 1.10
#define AXIS_SHIFT 2.0

using namespace std;
using namespace Eigen;

Thrower::Thrower(robotics::World &_world, robotics::Object &_object, wxTextCtrl *_timeText)
  : mWorld(_world), mObject(_object), mTimeText(_timeText), mRobotId(0) {
  // TODO: mRobotId shouldn't be fixed 0, rather a parameter
}

// The effect of this method is that it will fill the path value
void Thrower::throwObject(VectorXd pos, VectorXd vel) {
  VectorXd shift(3);
  shift << 0, 10, 0;
  
  //get random reachable position (rrp) for ball's projectile motion
  //...
  VectorXd rrp = findRandomReachablePosition(pos);
  
  //get random start position for ball's projectile motion
  //... 
  VectorXd startCoord(3);
  startCoord << 0, -3.6, 1.0; //area in front of arm
  VectorXd rstart = findRandomStartPosition(startCoord);
  
  //calculate velocities to pass through such (rrp) TODO
  //VectorXd vels = calculateVelocities(rstart, rrp);
  //PRINT(vels);
  
  //calculate motion in steps
  //objectPath = projectileMotion(rstart, vels, );
  objectPath = straightMotion(rstart, rrp);
  JointMover arm(mWorld, mRobotId);
  jointPath.clear();
  VectorXd joints = mWorld.getRobot(0)->getQuickDofs();
 
  for( list<VectorXd>::iterator it = objectPath.begin(); it != objectPath.end(); it++ ) {
    jointPath.push_back(joints);
    //joints = arm.OneStepTowardsXYZ(joints, *it);
  }
}

//get random double between fmin and fmax
double fRand(double min, double max)
{
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
}


VectorXd Thrower::findRandomReachablePosition(VectorXd pos){
       VectorXd target(3);
       
       srand(time(NULL));
       //get random reachable location; note generated z >= current z (i.e. arm is on table)
       target << fRand(pos[0] - ARM_LENGTH/2, pos[0] + ARM_LENGTH/2), 
                 fRand(pos[1] - ARM_LENGTH/2, pos[1] + ARM_LENGTH/2),
                 fRand(pos[2]+1, pos[2] + ARM_LENGTH/2);
       PRINT(target); 
       return target;
}

//NOTE: this method could be combined with the previous one, but might just better to have it separate
VectorXd Thrower::findRandomStartPosition(VectorXd pos){
        VectorXd start(3);
        srand(time(NULL));
        //get random start locations; chose a bounded random area in front of the arm
        start << fRand(pos[0] - AXIS_SHIFT, pos[0] + AXIS_SHIFT), 
                 fRand(pos[1] - AXIS_SHIFT, pos[1] + AXIS_SHIFT),
                 pos[2];
        PRINT(start); 
        return start;
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

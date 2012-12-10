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
#include "LinearPredictor.h"
#include "QuadraticPredictor.h"
#include "Predictor.h"
#include "PathPlanner.h"
#define PRINT(x) std::cout << #x << " = " << x << std::endl;
#define ECHO(x) std::cout << x << std::endl;
#define ARM_LENGTH 1.10
#define AXIS_SHIFT 2.0

using namespace std;
using namespace Eigen;

Thrower::Thrower(robotics::World &_world, wxTextCtrl *_timeText,
    robotics::Object &_sphereActual, robotics::Object &_spherePerceived, robotics::Object &_spherePredicted, robotics::Object &_aimStar) :
  mWorld(_world), mTimeText(_timeText), mRobotId(0),
  mSphereActual(_sphereActual), mSpherePerceived(_spherePerceived),  mSpherePredicted(_spherePredicted),
  mAimStar(_aimStar){
}


//get random double between fmin and fmax
double fRand(double min, double max) {
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
}


list<VectorXd> addSensorNoise(list<VectorXd> path, double maxNoise){
        for( list<VectorXd>::iterator it = path.begin(); it != path.end(); it++){
                VectorXd noise(3); noise << fRand(0.0,maxNoise), fRand(0.0,maxNoise), fRand(0.0,maxNoise);
                *it = *it + noise;
        }
        return path;        
}

VectorXd findRandomReachablePosition(VectorXd pos){
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
VectorXd findRandomStartPosition(VectorXd pos){
        VectorXd start(3);
        srand(time(NULL));
        //get random start locations; chose a bounded random area in front of the arm
        start << fRand(pos[0] - AXIS_SHIFT, pos[0] + AXIS_SHIFT), 
                 fRand(pos[1] - AXIS_SHIFT, pos[1] + AXIS_SHIFT),
                 pos[2];
        PRINT(start); 
        return start;
}

// The effect of this method is that it will fill the path value
void Thrower::throwObject(VectorXd pos, double noise, double prediction_time, int maxnodes, bool approach) {
       
  //get random reachable position (rrp) for ball's projectile motion
  VectorXd rrp = findRandomReachablePosition(pos);
  
  //get first a start coordinate to serve as center of bounded rectangle
  VectorXd startCoord(3); startCoord << 0, -2.6, 0.0; 
  
  //get random start position for ball's projectile motion
  VectorXd rstart = findRandomStartPosition(startCoord);
  
  //calculate velocities to pass through such (rrp)
  VectorXd vels = calculateVelocities(rstart, rrp);
  
  //define acceleration vector; accx=0, accy=0, accz= g
  VectorXd acc(3); acc<< 0,0,g;
  
  //calculate motion in steps
  objectPath = projectileMotion(rstart, vels, acc);
  perceivedPath = addSensorNoise(objectPath, noise);
  //objectPath = straightMotion(rstart, rrp);
  
  aims.clear();
  predictedPath.clear();
  predictedPaths.clear();

  for(list<VectorXd>::iterator it = perceivedPath.begin(); it != perceivedPath.end(); it++){
    list<VectorXd>::iterator it_after = it;
    it_after++;
    QuadraticPredictor predictor(list<VectorXd>(perceivedPath.begin(), it_after), prediction_time);
    predictedPaths.push_back(predictor.getPredictedPath());
    predictedPath.push_back(predictedPaths.back().back());
  }

  JointMover arm(mWorld, mRobotId);
  jointPath.clear();
  VectorXd joints = mWorld.getRobot(0)->getQuickDofs();
 
  bool GoToXYZ( VectorXd _qStart, VectorXd _targetXYZ, VectorXd &_qResult, double &_distance);
  for( list<Path>::iterator it_paths = predictedPaths.begin(); it_paths != predictedPaths.end(); it_paths++ ) {
    Path &path = *it_paths;
    VectorXd closestXYZ = path.back();
    VectorXd qClosest(7);
    qClosest << 360, 360, 360, 360, 360, 360, 36000000000000000;
    for( Path::iterator it = path.begin(); it != path.end(); it++ ) {
      double dist_now = JointMover::jointSpaceDistance(joints, qClosest);
      VectorXd qTemp;
      if(arm.GoToXYZ(joints, *it, qTemp)){
        double dist_other = JointMover::jointSpaceDistance(joints, qTemp);

        if(dist_other < dist_now){
          closestXYZ = *it;
          qClosest = qTemp;
        }
      }
    }
    jointPath.push_back(joints);
    VectorXd jointsGoal = qClosest.norm() > 100000 ? joints : qClosest;
    
    //create path planner
    PathPlanner planner(mWorld, false, jointSpeeds*dt);
    VectorXi mLinks = mWorld.getRobot(mRobotId)->getQuickDofsIndices();
    
    if(planner.planPath(mRobotId,mLinks, joints, jointsGoal, false, false, true, false, maxnodes)){ 
        list<VectorXd>::iterator ite = planner.path.begin();
        joints = *(++ite);
    }
    //joints = JointMover::jointSpaceMovement(joints, jointsGoal);
    aims.push_back(closestXYZ);
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
    list<VectorXd>::iterator it_pred;
    list<VectorXd>::iterator it_percvd;
    list<VectorXd>::iterator it_aim;
    for( list<VectorXd>::iterator it = objectPath.begin(), it_j = jointPath.begin(),it_percvd = perceivedPath.begin(), it_pred = predictedPath.begin(), it_aim = aims.begin();
         it != objectPath.end() && it_pred != predictedPath.end(); it++, it_j++, it_percvd++, it_pred++, it_aim++ ) {
        
        VectorXd &pos = *it;
        VectorXd &pos_pred = *it_pred;
        VectorXd &pos_percvd = *it_percvd;
        VectorXd &pos_aim = *it_aim;
        
        //update sphere in motion
        mSphereActual.setPositionXYZ(pos[0], pos[1], pos[2]);
        mSphereActual.update();
        
        //update perceived sphere
        mSpherePerceived.setPositionXYZ(pos_percvd[0], pos_percvd[1], pos_percvd[2]);
        mSpherePerceived.update();
        
        //update predicted sphere position
        mSpherePredicted.setPositionXYZ(pos_pred[0], pos_pred[1], pos_pred[2]);
        mSpherePredicted.update();
        
        //update robot's calculation of where to intercept sphere
        mAimStar.setPositionXYZ(pos_aim[0], pos_aim[1], pos_aim[2]);
        mAimStar.update();
        
        //update robot and world frame
        mWorld.getRobot(mRobotId)->setQuickDofs( *it_j );
        mWorld.getRobot(mRobotId)->update();
        frame->AddWorld( &mWorld );
    }
}

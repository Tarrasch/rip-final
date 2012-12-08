#include <iostream>
#include <stdlib.h>
#include <Eigen/LU>
#include <Eigen/Core>
#include <vector>
#include <list>
#include <robotics/World.h>
#include <wx/wx.h>
#include "LinearPredictor.h"
#define PRINT(x) std::cout << #x << " = " << x << std::endl;
#define ECHO(x) std::cout << x << std::endl;

using namespace std;
using namespace Eigen;

LinearPredictor::LinearPredictor(std::list<Eigen::VectorXd> observedPath){
        VectorXd last = observedPath.back();
        list<VectorXd>::iterator it = observedPath.end();
        it--;
        VectorXd beforeLast = *it;
        
        PRINT(last);
        PRINT(beforeLast);
        
        //call projectileMotionWRandT with no randomness
        VectorXd vel = (last - beforeLast);
        VectorXd acc(3); acc<<0,0,0;
        predictedPath = projectileMotionWRandT(beforeLast, vel, acc, 0, 4);
}
    
std::list<Eigen::VectorXd> LinearPredictor::getPredictedPath(){
        return predictedPath;
}

void LinearPredictor::SetPredictorTimeline(){
    if( predictedPath.size() == 0 ) {
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




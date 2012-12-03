#include <stdlib.h>
#include "Thrower.h"
#include "newtonianPhysics.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/GRIPTab.h>

using namespace std;
using namespace Eigen;

Thrower::Thrower(robotics::World &_world, robotics::Object &_object, wxTextCtrl *_timeText)
  : mWorld(_world), mObject(_object), mTimeText(_timeText) {

}

// The effect of this method is that it will fill the path value
void Thrower::throwObject(VectorXd pos, VectorXd vel) {
  objectPath = projectileMotion(pos, vel);
}

void Thrower::SetThrowTimeline(){
    if( objectPath.size() == 0 ) {
        cout << "--(!) Must create a nonempty plan before setting dimeline (!)--" << endl;
        return;
    }

    double T;
    mTimeText->GetValue().ToDouble(&T);

    int numsteps = objectPath.size();
    double increment = T/(double)numsteps;
    cout << "-->(+) Updating Timeline - Increment: " << increment
         << " Total T: " << T << " Steps: " << numsteps << endl;

    frame->InitTimer( string("Throwing of object"),increment );

    for( list<VectorXd>::iterator it = objectPath.begin(); it != objectPath.end(); it++ ) {
        VectorXd &pos = *it;
        mObject.setPositionXYZ(pos[0], pos[1], pos[2]);
				mObject.update();
        frame->AddWorld( &mWorld );
    }

}

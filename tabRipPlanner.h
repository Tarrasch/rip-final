/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#ifndef RIP_PLANNER_TAB
#define RIP_PLANNER_TAB

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>

#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Joint.h>
#include "PathPlanner.h"
#include <kinematics/Dof.h> 

#include <iostream>

const double STOP_THRESHOLD = 0.12;
/**
 * @class RipTabPlanner
 * @brief Implements the RIP Tab + Planners
 */
class RipPlannerTab : public GRIPTab
{
public:
    Eigen::VectorXd mStartConf;
    Eigen::VectorXd mGoalConf;
    
    int mRobotId;
    Eigen::VectorXi mLinks;

    int mRrtStyle;
    bool mGreedyMode;
    bool mConnectMode;
    bool mSmooth;
    bool mApproach;
    PathPlanner *mPlanner;

    wxTextCtrl *mTimeText;
    wxTextCtrl *mNoiseText;
    wxTextCtrl *mIterationsText;
    wxTextCtrl *mPredictorText;
    wxTextCtrl *mPredTimeText;
    wxTextCtrl *mNodesText;

    // public vars to capture external selection stuff
    robotics::Object* mSelectedObject;
    robotics::Robot* mSelectedRobot;
    dynamics::BodyNodeDynamics* mSelectedNode;

    /// Functions

    RipPlannerTab(){};
    RipPlannerTab( wxWindow * parent, wxWindowID id = -1,
                   const wxPoint & pos = wxDefaultPosition,
                   const wxSize & size = wxDefaultSize,
                   long style = wxTAB_TRAVERSAL);
    virtual ~RipPlannerTab(){}

    void OnSlider(wxCommandEvent &evt);
    void OnRadio(wxCommandEvent &evt);
    void OnButton(wxCommandEvent &evt);
    void OnCheckBox(wxCommandEvent &evt);
    void SetTimeline();
    void GRIPStateChange();
    
    // Thread specific
    // GRIPThread* thread;

    // Your Thread routine
    // call GRIPThread::CheckPoint() regularly
    // void Thread();
    // void onCompleteThread();

    DECLARE_DYNAMIC_CLASS( RipTabPlannerTab )
    DECLARE_EVENT_TABLE()
};

#endif /** RIP_PLANNER_TAB */

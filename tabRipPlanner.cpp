/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include "tabRipPlanner.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/GRIPTab.h>

#include <iostream>

#include <Tabs/AllTabs.h>
#include <GRIPApp.h>
#include "JTFollower/JTFollower.h" 
#include "Thrower.h" 
#define PRINT(x) std::cout << #x << " = " << x << std::endl;
#define ECHO(x) std::cout << x << std::endl;


/* Quick intro to adding tabs:
 * 1- Copy template cpp and header files and replace with new class name
 * 2- include classname.h in AllTabs.h, and use the ADD_TAB macro to create it
 */

// Control IDs (used for event handling - be sure to start with a non-conflicted id)
enum planTabEvents {
	button_SetStart = 50,
	button_SetGoal,
	button_showStart,
	button_showGoal,
	button_resetPlanner,
	button_empty1,
	button_empty2,
	button_Plan,
	button_Stop,
	button_UpdateTime,
	button_ExportSequence,
	button_ShowPath,
	checkbox_beGreedy,
	checkbox_useConnect,
	checkbox_useSmooth,
	slider_Time,
  button_TestThrow

};

// sizer for whole tab
wxBoxSizer* sizerFull;

//Add a handler for any events that can be generated by the widgets you add here (sliders, radio, checkbox, etc)
BEGIN_EVENT_TABLE(RipPlannerTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, RipPlannerTab::OnSlider)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_RADIOBOX_SELECTED, RipPlannerTab::OnRadio)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, RipPlannerTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_CHECKBOX_CLICKED, RipPlannerTab::OnCheckBox)
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of RSTTab
IMPLEMENT_DYNAMIC_CLASS(RipPlannerTab, GRIPTab)

/**
 * @function RipTabPlanner
 * @brief Constructor
 */
RipPlannerTab::RipPlannerTab( wxWindow *parent, const wxWindowID id,
		              const wxPoint& pos, const wxSize& size, long style) :
	                      GRIPTab(parent, id, pos, size, style) {

    mStartConf.resize(0);
    mGoalConf.resize(0);

    mRobotId = 0;
    mLinks.resize(0);

    mRrtStyle = 0;
    mGreedyMode = false;
    mConnectMode = false;
    mSmooth = false;
    mPlanner = NULL;

    sizerFull = new wxBoxSizer( wxHORIZONTAL );

    // ** Create left static box for configuring the planner **

    // Create StaticBox container for all items
    wxStaticBox* configureBox = new wxStaticBox(this, -1, wxT("Configure"));

    // Create sizer for this box with horizontal layout
    wxStaticBoxSizer* configureBoxSizer = new wxStaticBoxSizer(configureBox, wxHORIZONTAL);

    // Create a sizer for radio buttons in 1st column
    wxBoxSizer *col1Sizer = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer *miniSizer = new wxBoxSizer(wxVERTICAL); // annoying hack to get checkboxes close together
    miniSizer->Add( new wxCheckBox(this, checkbox_beGreedy, _T("&goal bias (be greedy)")),
		    1, // vertical stretch evenly
		    wxALIGN_NOT,
		    0);
    miniSizer->Add( new wxCheckBox(this, checkbox_useConnect, _T("use &connect algorithm (be really greedy)")),
		    1, // vertical stretch evenly
		    wxALIGN_NOT,
		    0 );
    miniSizer->Add( new wxCheckBox(this, checkbox_useSmooth, _T("use &smoother (make it less ugly)")),
		    1, // vertical stretch evenly
		    wxALIGN_NOT,
		    0 );
    col1Sizer->Add(miniSizer,1,wxALIGN_NOT,0);

    // Create radio button for rrt_style
    static const wxString RRTStyles[] =
    {
        wxT("Single"),
	wxT("Bi-directional")
    };
    col1Sizer->Add( new wxRadioBox(this, wxID_ANY, wxT("RRT &style:"),
		    wxDefaultPosition, wxDefaultSize, WXSIZEOF(RRTStyles), RRTStyles, 1,
		    wxRA_SPECIFY_ROWS),
		    1, // stretch evenly with buttons and checkboxes
		    wxALIGN_NOT,
		    0 );
    // Add col1 to configureBoxSizer
    configureBoxSizer->Add( col1Sizer,
			    3, // 3/5 of configure box
			    wxALIGN_NOT,
			    0 ); //

    // Create sizer for start buttons in 2nd column
    wxBoxSizer *col2Sizer = new wxBoxSizer(wxVERTICAL);
    col2Sizer->Add( new wxButton(this, button_SetStart, wxT("Set &Start")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    col2Sizer->Add( new wxButton(this, button_showStart, wxT("Show S&tart")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    col2Sizer->Add( new wxButton(this, button_empty1, wxT("Check collision")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together


    // Add col2Sizer to the configuration box
    configureBoxSizer->Add( col2Sizer,
			    1, // takes half the space of the configure box
			    wxALIGN_NOT ); // no border and center horizontally

    // Create sizer for goal buttons in 3rd column
    wxBoxSizer *col3Sizer = new wxBoxSizer(wxVERTICAL);
    col3Sizer->Add( new wxButton(this, button_SetGoal, wxT("Set &Goal")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    col3Sizer->Add( new wxButton(this, button_showGoal, wxT("Show G&oal")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    col3Sizer->Add( new wxButton(this, button_empty2, wxT("Empty 2")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    // HACK, try add throw button
    col3Sizer->Add( new wxButton(this, button_TestThrow, wxT("Test Throw")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    configureBoxSizer->Add( col3Sizer,
			    1, // size evenly with radio box and checkboxes
			    wxALIGN_NOT ); // no border and center horizontally

    // Add this box to parent sizer
    sizerFull->Add( configureBoxSizer,
		    4, // 4-to-1 ratio with execute sizer, since it just has 3 buttons
		    wxEXPAND | wxALL,
		    6 );


    // ** Create right static box for running the planner **
    wxStaticBox* executeBox = new wxStaticBox(this, -1, wxT("Execute Planner"));

    // Create sizer for this box
    wxStaticBoxSizer* executeBoxSizer = new wxStaticBoxSizer(executeBox, wxVERTICAL);

    // Add buttons for "plan", "save movie", and "show path"
    executeBoxSizer->Add( new wxButton(this, button_Plan, wxT("&Start")),
	 		  1, // stretch to fit horizontally
			  wxGROW ); // let it hog all the space in it's column

    executeBoxSizer->Add( new wxButton(this, button_Stop, wxT("&Stop")),
			  1, // stretch to fit horizontally
			  wxGROW );


    wxBoxSizer *timeSizer = new wxBoxSizer(wxHORIZONTAL);
    mTimeText = new wxTextCtrl(this,1008,wxT("5.0"),wxDefaultPosition,wxSize(40,20),wxTE_RIGHT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);
    timeSizer->Add( mTimeText,2,wxALL,1 );
    timeSizer->Add(new wxButton(this, button_UpdateTime, wxT("Set T(s)")),2,wxALL,1);
    executeBoxSizer->Add(timeSizer,1,wxALL,2);

    executeBoxSizer->Add( new wxButton(this, button_ShowPath, wxT("&Print")),
			  1, // stretch to fit horizontally
			  wxGROW );

    sizerFull->Add(executeBoxSizer, 1, wxEXPAND | wxALL, 6);

    SetSizer(sizerFull);

}

/**
 * @function OnRadio
 * @brief Handle Radio toggle
 */
void RipPlannerTab::OnRadio(wxCommandEvent &evt) {

	mRrtStyle = evt.GetSelection();
	std::cout << "rrtStyle = " << mRrtStyle << std::endl;
}

/**
 * @function OnButton
 * @brief Handle Button Events
 */
void RipPlannerTab::OnButton(wxCommandEvent &evt) {

  int button_num = evt.GetId();

  switch (button_num) {

    /** Set Start */
  case button_SetStart:
    if ( mWorld != NULL ) {
      if( mWorld->getNumRobots() < 1) {
	std::cout << "(!) Must have a world with a robot to set a Start state" << std::endl;
	break;
      }
      std::cout << "(i) Setting Start state for " << mWorld->getRobot(mRobotId)->getName() << ":" << std::endl;

      mStartConf = mWorld->getRobot(mRobotId)->getQuickDofs();
     
      for( unsigned int i = 0; i < mStartConf.size(); i++ )
	{  std::cout << mStartConf(i) << " ";  }
      std::cout << std::endl;
    } else {
      std::cout << "(!) Must have a world loaded to set a Start state." << std::endl;
    }
    break;

    /** Set Goal */
  case button_SetGoal:
    if ( mWorld != NULL ) {
      if( mWorld->getNumRobots() < 1){
	std::cout << "(!) Must have a world with a robot to set a Goal state.(!)" << std::endl;
	break;
      }
      std::cout << "(i) Setting Goal state for " << mWorld->getRobot(mRobotId)->getName() << ":" << std::endl;

      mGoalConf = mWorld->getRobot(mRobotId)->getQuickDofs();

      for( unsigned int i = 0; i < mGoalConf.size(); i++ )
	{ std::cout << mGoalConf(i) << " "; }
      std::cout << std::endl;
    } else {
      std::cout << "(!) Must have a world loaded to set a Goal state"<< std::endl;
    }
    break;

    /** Show Start */
  case button_showStart:
    if( mStartConf.size() < 1 ){
      std::cout << "(x) First, set a start configuration" << std::endl;
      break;
    }

    mWorld->getRobot(mRobotId)->setQuickDofs( mStartConf );
   
    for( unsigned int i = 0; i< mStartConf.size(); i++ )
      {  std::cout << mStartConf(i) << " "; }
    std::cout << std::endl;

    mWorld->getRobot(mRobotId)->update();
    viewer->UpdateCamera();
    break;

    /** Show Goal */
  case button_showGoal:
    if( mGoalConf.size() < 1 ){
      std::cout << "(x) First, set a goal configuration" << std::endl;
      break;
    }

    mWorld->getRobot(mRobotId)->setQuickDofs( mGoalConf );
    
    for( unsigned int i = 0; i< mGoalConf.size(); i++ )
      {  std::cout << mGoalConf[i] << " ";  }
    std::cout << std::endl;

    mWorld->getRobot(mRobotId)->update();
    viewer->UpdateCamera();
    break;

    /** Reset Planner */
  case button_resetPlanner:
    if ( mWorld != NULL) {
      if ( mPlanner != NULL)
	delete mPlanner;

      std::cout << "Creating a new planner" << std::endl;
      double stepSize = 0.1; // default
      mPlanner = new PathPlanner( *mWorld, false, stepSize );
    } else {
      std::cout << "(!) Must have a world loaded to make a planner" << std::endl;
    }
    break;

    /** Empty button 1 */
  case button_empty1:
    {
      std::cout << "(0) Checking Collisions" << std::endl;
      bool st;
      st = mWorld->checkCollision();
      if( st == true )
	{ printf("Collisions \n");}
      else
	{ printf("No Collisions \n");}

    }
    break;

    /** Empty button 2 */
  case button_empty2:
    std::cout << "-- (0) Empty Button to use for whatever you want (0)--" << std::endl;
    break;

    /** Execute Plan */
  case button_Plan:
    {
      if( mGoalConf.size() < 0 ){ std::cout << "(x) Must set a goal" << std::endl; break; }
      if( mStartConf.size() < 0 ){ std::cout << "(x) Must set a start" << std::endl; break; }
      if( mWorld == NULL ){ std::cout << "(x) Must load a world" << std::endl; break; }
      if( mWorld->getNumRobots() < 1){ std::cout << "(x) Must load a world with a robot" << std::endl; break; }

      double stepSize = 0.02;

      mPlanner = new PathPlanner( *mWorld, false, stepSize );
      //wxThread planThread;
      //planThread.Create();
      
      mLinks = mWorld->getRobot(mRobotId)->getQuickDofsIndices();
      
      int maxNodes = 5000;
      bool results = mPlanner->planPath( mRobotId,
					mLinks,
					mStartConf,
					mGoalConf,
					mRrtStyle,
					mConnectMode,
					mGreedyMode,
					mSmooth,
					maxNodes );
		  
      if( results )
	{  SetTimeline(); }
    }
    break;

    /** Update Time */
  case button_UpdateTime:
    {
      /// Update the time span of the movie timeline
      SetTimeline();
    }
    break;

    /** Show Path */
  case button_ShowPath:
    if( mWorld == NULL || mPlanner == NULL || mPlanner->path.size() == 0 ) {
      std::cout << "(!) Must create a valid plan before printing."<<std::endl;
      return;
    } else {
      std::cout<<"(i) Printing...Implement me :)"<<std::endl;
    }
    break;

  case button_TestThrow:
	ECHO("Test Throw Pressed");
	
    if ( mWorld != NULL ) {
        robotics::Object* sphere_red   = mWorld->getObject(mWorld->getNumObjects()-3); // TODO: Fix prettier
        robotics::Object* sphere_blue  = mWorld->getObject(mWorld->getNumObjects()-2); // TODO: Fix prettier
        robotics::Object* sphere_green = mWorld->getObject(mWorld->getNumObjects()-1); // TODO: Fix prettier
        Thrower thrower(*mWorld, mTimeText, *sphere_red, *sphere_blue, *sphere_green);
        
        //find robot position to determine direction of sphere_red
        VectorXd robotPos(3);
        //get current robot position
        double x,y,z;
        mWorld->getRobot(mRobotId)->getPositionXYZ(x,y,z);
        robotPos << x, y, z;

        //the target position will be a random location reachable by the robot arm
        //velocities are calculated to reach such position
        thrower.throwObject(robotPos);
        thrower.SetThrowTimeline();
    } else {
      std::cout << "(!) World must be loaded!!!!!!!!!!!"<< std::endl;
    }
    break;

  } // end of case
}

/**
 * @function setTimeLine
 * @brief
 */
void RipPlannerTab::SetTimeline() {

    if( mWorld == NULL || mPlanner == NULL || mPlanner->path.size() == 0 ) {
        cout << "--(!) Must create a valid plan before updating its duration (!)--" << endl;
	return;
    }

    double T;
    mTimeText->GetValue().ToDouble(&T);

    int numsteps = mPlanner->path.size();
    double increment = T/(double)numsteps;

    cout << "-->(+) Updating Timeline - Increment: " << increment << " Total T: " << T << " Steps: " << numsteps << endl;

    frame->InitTimer( string("RRT_Plan"),increment );

    Eigen::VectorXd vals( mLinks.size() );

    for( std::list<Eigen::VectorXd>::iterator it = mPlanner->path.begin(); it != mPlanner->path.end(); it++ ) {

        mWorld->getRobot( mRobotId)->setQuickDofs( *it );
				mWorld->getRobot(mRobotId)->update();
        
        frame->AddWorld( mWorld );
    }

}

/**
 * @function OnCheckBox
 * @brief Handle CheckBox Events
 */
void RipPlannerTab::OnCheckBox( wxCommandEvent &evt ) {
  int checkbox_num = evt.GetId();

  switch (checkbox_num) {

  case checkbox_beGreedy:
    mGreedyMode = (bool)evt.GetSelection();
    std::cout << "(i) greedy = " << mGreedyMode << std::endl;
    break;

  case checkbox_useConnect:
    mConnectMode = (bool)evt.GetSelection();
    std::cout << "(i) useConnect = " << mConnectMode << std::endl;
    break;
  case checkbox_useSmooth:
    mSmooth = (bool)evt.GetSelection();
    std::cout << "(i) Smooth option = " << mSmooth << std::endl;
    break;
  }
}

/**
 * @function OnSlider
 * @brief Handle slider changes
 */
void RipPlannerTab::OnSlider(wxCommandEvent &evt) {
  if (selectedTreeNode == NULL) {
    return;
  }

  int slnum = evt.GetId();
  double pos = *(double*) evt.GetClientData();
  char numBuf[1000];

  switch (slnum) {
  case slider_Time:
    sprintf(numBuf, "X Change: %7.4f", pos);
    std::cout << "(i) Timeline slider output: " << numBuf << std::endl;
    //handleTimeSlider(); // uses slider position to query plan state
    break;

  default:
    return;
  }
  //world->updateCollision(o);
  //viewer->UpdateCamera();

  if (frame != NULL)
    frame->SetStatusText(wxString(numBuf, wxConvUTF8));
}

/**
 * @function GRIPStateChange
 * @brief This function is called when an object is selected in the Tree View or other
 *        global changes to the RST world. Use this to capture events from outside the tab.
 */
void RipPlannerTab::GRIPStateChange() {
  if ( selectedTreeNode == NULL ) {
    return;
  }

  std::string statusBuf;
  std::string buf, buf2;

  switch (selectedTreeNode->dType) {

  case Return_Type_Object:
    mSelectedObject = (robotics::Object*) ( selectedTreeNode->data );
    statusBuf = " Selected Object: " + mSelectedObject->getName();
    buf = "You clicked on object: " + mSelectedObject->getName();

    // Enter action for object select events here:

    break;
  case Return_Type_Robot:
    mSelectedRobot = (robotics::Robot*) ( selectedTreeNode->data );
    statusBuf = " Selected Robot: " + mSelectedRobot->getName();
    buf = " You clicked on robot: " + mSelectedRobot->getName();

    // Enter action for Robot select events here:

    break;
  case Return_Type_Node:
    mSelectedNode = (dynamics::BodyNodeDynamics*) ( selectedTreeNode->data );
    statusBuf = " Selected Body Node: " + string(mSelectedNode->getName()) + " of Robot: "
      + ( (robotics::Robot*) mSelectedNode->getSkel() )->getName();
    buf = " Node: " + std::string(mSelectedNode->getName()) + " of Robot: " + ( (robotics::Robot*) mSelectedNode->getSkel() )->getName();

    // Enter action for link select events here:

    break;
  default:
    fprintf(stderr, "--( :D ) Someone else's problem!\n");
    assert(0);
    exit(1);
  }

  //cout << buf << endl;
  frame->SetStatusText(wxString(statusBuf.c_str(), wxConvUTF8));
  sizerFull->Layout();
}

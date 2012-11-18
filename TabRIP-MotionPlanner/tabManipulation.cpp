/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include "tabManipulation.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/GRIPTab.h>

#include <iostream>

#include <Tabs/AllTabs.h>
#include <GRIPApp.h>
#include "JTQuickFind.h" 
#define PRINT(x) std::cout << #x << " = " << x << std::endl;

// Control IDs (used for event handling - be sure to start with a non-conflicted id)
enum planTabEvents {
	button_SetStart = 50,
	button_SetGoal,
	button_showStart,
	button_showGoal,
	button_resetPlanner,
	button_collision,
	button_Plan,
	button_Stop,
	button_UpdateTime,
	button_ExportSequence,
	button_ShowPath,
	checkbox_beGreedy,
	checkbox_useConnect,
	checkbox_useSmooth,
	slider_Time
};

// sizer for whole tab
wxBoxSizer* sizerIsFull;

//Add a handler for any events that can be generated by the widgets you add here (sliders, radio, checkbox, etc)
BEGIN_EVENT_TABLE(ManipulationTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, ManipulationTab::OnSlider)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_RADIOBOX_SELECTED, ManipulationTab::OnRadio)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, ManipulationTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_CHECKBOX_CLICKED, ManipulationTab::OnCheckBox)
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of RSTTab
IMPLEMENT_DYNAMIC_CLASS(ManipulationTab, GRIPTab)

/**
 * @function ManipulationTab
 * @brief Constructor
 */
ManipulationTab::ManipulationTab( wxWindow *parent, const wxWindowID id,
		              const wxPoint& pos, const wxSize& size, long style) :
	                      GRIPTab(parent, id, pos, size, style) {

    mStartConf.resize(0);

    mRobotId = 0;
    mLinks.resize(0);

    mRrtStyle = 0;
    mGreedyMode = false;
    mConnectMode = false;
    mSmooth = false;
    mFirstPlanner = NULL;
    mSecondPlanner = NULL;
    
    sizerIsFull = new wxBoxSizer( wxHORIZONTAL );

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
    col2Sizer->Add( new wxButton(this, button_collision, wxT("Check collision")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together


    // Add col2Sizer to the configuration box
    configureBoxSizer->Add( col2Sizer,
			    1, // takes half the space of the configure box
			    wxALIGN_NOT ); // no border and center horizontally

    // Create sizer for goal buttons in 3rd column
    wxBoxSizer *col3Sizer = new wxBoxSizer(wxVERTICAL);
    col3Sizer->Add( new wxButton(this, button_SetGoal, wxT("Set Object Goal")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
		    
		col3Sizer->Add( new wxButton(this, button_showGoal, wxT("Show Object Goal")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
		    
    configureBoxSizer->Add( col3Sizer,
			    1, // size evenly with radio box and checkboxes
			    wxALIGN_NOT ); // no border and center horizontally

    // Add this box to parent sizer
    sizerIsFull->Add( configureBoxSizer,
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

    sizerIsFull->Add(executeBoxSizer, 1, wxEXPAND | wxALL, 6);

    SetSizer(sizerIsFull);

}

/**
 * @function OnRadio
 * @brief Handle Radio toggle
 */
void ManipulationTab::OnRadio(wxCommandEvent &evt) {

	mRrtStyle = evt.GetSelection();
	std::cout << "rrtStyle = " << mRrtStyle << std::endl;
}

/**
 * @function OnButton
 * @brief Handle Button Events
 */
void ManipulationTab::OnButton(wxCommandEvent &evt) {

  int button_num = evt.GetId();

  switch (button_num) {

    /** Set Start */
  case button_SetStart:
    if ( mWorld != NULL) {
      if( mWorld->getNumRobots() < 1) {
	std::cout << "(!) Must have a world with a robot to set a Start state" << std::endl;
	break;
      }
      std::cout << "(i) Setting Start state for " << mWorld->getRobot(mRobotId)->getName() << " and " << mSelectedObject->getName() << std::endl;

      mStartConf = mWorld->getRobot(mRobotId)->getQuickDofs();
      if(mSelectedObject == NULL){
        std::cout << "(!) Must first select an object" << std::endl;
        break;
      }
      //set start state for object selected
      mSelectedObject->getPositionXYZ(start_x,start_y,start_z);
      object = mSelectedObject;
      object->update();
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
      std::cout << "(i) Setting Goal state for " << mSelectedObject->getName() << ":" << std::endl;
      
      //set goal for object; saved by reference in variables goal_x,y,z
      if(mSelectedObject == NULL){
        std::cout << "(!) Must first select an object" << std::endl;
        break;
      }
      mSelectedObject->getPositionXYZ(goal_x,goal_y,goal_z);
    } else {
      std::cout << "(!) Must have a world loaded to set a Goal state"<< std::endl;
    }
    break;

    /** Show Start */
  case button_showStart:
    if( mStartConf.size() < 1 || object == NULL){
      std::cout << "(x) First, set a start configuration" << std::endl;
      break;
    }
    mWorld->getRobot(mRobotId)->setQuickDofs( mStartConf );
    object->setPositionXYZ(start_x,start_y,start_z);
    
    for( unsigned int i = 0; i< mStartConf.size(); i++ )
      {  std::cout << mStartConf(i) << " "; }
    std::cout << std::endl;

    mWorld->getRobot(mRobotId)->update();
    object->update();
    viewer->UpdateCamera();
    break;

    /** Show Goal */
  case button_showGoal:
    if(object == NULL){
      std::cout << "(x) First, set a goal for object" << std::endl;
      break;
    }
    //show goal position for object
    object->setPositionXYZ(goal_x,goal_y,goal_z);
    object->update();
    viewer->UpdateCamera();
    break;

    /** Reset Planner */
  case button_resetPlanner:
    if ( mWorld != NULL) {
      if ( mFirstPlanner != NULL && mSecondPlanner != NULL){
	        delete mFirstPlanner;
	        delete mSecondPlanner;
	}

      std::cout << "Creating a new planner" << std::endl;
      double stepSize = 0.1; // default
      mFirstPlanner = new PathPlanner( *mWorld, false, stepSize );
      mSecondPlanner = new PathPlanner( *mWorld, false, stepSize );
    } else {
      std::cout << "(!) Must have a world loaded to make a planner" << std::endl;
    }
    break;

    /** Collision button */
  case button_collision:
    {
      if( mWorld == NULL){
      std::cout << "(x) First, load a world and set a start/goal configuration" << std::endl;
      break;
    } 
      std::cout << "(0) Checking Collisions" << std::endl;
      bool st;
      st = mWorld->checkCollision();
      if( st == true )
	{ printf("Collisions \n");}
      else
	{ printf("No Collisions \n");}

    }
    break;

    /** Execute Plan */
  case button_Plan:
    {
      if( goal_x == NULL ){ std::cout << "(x) Must set a goal for object" << std::endl; break; }
      if( mStartConf.size() < 0 ){ std::cout << "(x) Must set a start" << std::endl; break; }
      if( mWorld == NULL ){ std::cout << "(x) Must load a world" << std::endl; break; }
      if( mWorld->getNumRobots() < 1){ std::cout << "(x) Must load a world with a robot" << std::endl; break; }

      double stepSize = 0.02;
      
      mFirstPlanner = new PathPlanner( *mWorld, false, stepSize );
      mSecondPlanner = new PathPlanner( *mWorld, false, stepSize );
      
      mLinks = mWorld->getRobot(mRobotId)->getQuickDofsIndices();
      
      //get first goal configuration
      JTQuickFind *goal_finder = new JTQuickFind(*mWorld); 
      
      //get data to init goal_finder
      int mNumLinks = mWorld->getRobot(mRobotId)->getNumQuickDofs(); 
      int EEDofId = mLinks( mNumLinks - 1 );
      int mEEId = mWorld->getRobot(mRobotId)->getDof( EEDofId )->getJoint()->getChildNode()->getSkelIndex();
      std::string mEEName =  mWorld->getRobot(mRobotId)->getNode(mEEId)->getName();
      //init goal_finder with robot info
      goal_finder->init( mRobotId, mLinks, mEEName , mEEId, 0.02 ); 
      
      //get coordinates for robot start position
      Eigen::VectorXd startXYZ = goal_finder->GetXYZ(mStartConf);
      Eigen::VectorXd mPartialXYZ;
      mPartialXYZ.resize(3);
      //calculate adjusted coordinates for target position,i.e, location of object
	    mPartialXYZ << start_x, start_y, start_z; 
	    
      mPartialXYZ = (mPartialXYZ - startXYZ) * 0.8 + startXYZ;
      
      
      Eigen::VectorXd start = mStartConf;
      Eigen::VectorXd mPartialConf; 
 
      //find joint angles values given target position
      if( goal_finder->GoToXYZ( start, mPartialXYZ ) == true ){
		       mPartialConf = start;
		  }
		  
		  //perform initial plan to get to object
      int maxNodes = 5000;
      bool result_first = mFirstPlanner->planPath( mRobotId,
					mLinks,
					mStartConf,
					mPartialConf,
					mRrtStyle,
					mConnectMode,
					mGreedyMode,
					mSmooth,
					maxNodes );
			
			Eigen::VectorXd mFinalXYZ;
			mFinalXYZ.resize(3);
			mFinalXYZ << goal_x, goal_y, goal_z;
			Eigen::VectorXd mFinalConf;
			
			mFinalXYZ = (mFinalXYZ - mPartialXYZ) * 0.8 + mPartialXYZ;
			PRINT("FINISHED FIRST PLANNING");
			//attach object to end-effector
			AttachObject();
			
			if( goal_finder->GoToXYZ(start, mFinalXYZ) == true){
			   mFinalConf = start;
			}
			
			//perform final plan to get to object
			bool result_second = mSecondPlanner->planPath( mRobotId,
					mLinks,
					mPartialConf,
					mFinalConf,
					mRrtStyle,
					mConnectMode,
					mGreedyMode,
					mSmooth,
					maxNodes );
			
			PRINT(result_second);
			PRINT("FINISHED SECOND PLANNING");
      if( result_first && result_second  )
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
    if( mWorld == NULL || mFirstPlanner == NULL || mFirstPlanner->path.size() == 0 ) {
      std::cout << "(!) Must create a valid plan before printing."<<std::endl;
      return;
    } else {
      std::cout<<"(i) Printing...Implement me :)"<<std::endl;
    }
    break;
  }
}

void ManipulationTab::AttachObject(){
  object->setMovable(true);
  kinematics::BodyNode *last = mWorld->getRobot(mRobotId)->getNode("FT");
  kinematics::Joint *obj_joint = new kinematics::Joint(last, object->getNode(1), "object_joint");
  mWorld->getRobot(mRobotId)->addJoint(obj_joint);
  mWorld->getRobot(mRobotId)->addNode(object->getNode(1)); 
  mWorld->getRobot(mRobotId)->update();
  object->update();
}

/**
 * @function setTimeLine
 * @brief
 */
void ManipulationTab::SetTimeline() {

    if( mWorld == NULL || mFirstPlanner == NULL || mSecondPlanner == NULL || mFirstPlanner->path.size() == 0 || mSecondPlanner->path.size() == 0 ) {
        cout << "--(!) Must create a valid plan before updating its duration (!)--" << endl;
	return;
    }

    double T;
    mTimeText->GetValue().ToDouble(&T);

    int numsteps = mFirstPlanner->path.size() + mSecondPlanner->path.size();
    double increment = T/(double)numsteps;

    cout << "-->(+) Updating Timeline - Increment: " << increment << " Total T: " << T << " Steps: " << numsteps << endl;

    frame->InitTimer( string("RRT_Plan"),increment );

    Eigen::VectorXd vals( mLinks.size() );
    
    //fill timeline first with first planner results
    for( std::list<Eigen::VectorXd>::iterator it = mFirstPlanner->path.begin(); it != mFirstPlanner->path.end(); it++ ) {

        mWorld->getRobot( mRobotId)->setQuickDofs( *it );
				mWorld->getRobot(mRobotId)->update();
        
        frame->AddWorld( mWorld );
    }
    //now, with second planner results
    for( std::list<Eigen::VectorXd>::iterator it = mSecondPlanner->path.begin(); it != mSecondPlanner->path.end(); it++ ) {

        mWorld->getRobot( mRobotId)->setQuickDofs( *it );
				mWorld->getRobot(mRobotId)->update();
        
        frame->AddWorld( mWorld );
    }

}

/**
 * @function OnCheckBox
 * @brief Handle CheckBox Events
 */
void ManipulationTab::OnCheckBox( wxCommandEvent &evt ) {
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
void ManipulationTab::OnSlider(wxCommandEvent &evt) {
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
void ManipulationTab::GRIPStateChange() {
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
  sizerIsFull->Layout();
}
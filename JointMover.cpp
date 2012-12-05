/**
 * @file JointMover.cpp
 * @brief Returns joint angle information for a desired goal position given in X,Y and Z
 * @author Juan C. Garcia made minor modifications to code provided by Ana C. Huaman Quispe.
 */

#include <iostream>
#include <stdlib.h>
#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>

#include <Eigen/LU>
#include "JointMover.h"

using namespace std;
using namespace Eigen;

JointMover::JointMover( robotics::World &_world, int _robotId, double _configStep)
  : mConfigStep(_configStep), mWorld(_world), mRobotId(_robotId) {

  mLinks = mWorld.getRobot(mRobotId)->getQuickDofsIndices();
  //get data to initialize
  
  int mNumLinks = mWorld.getRobot(mRobotId)->getNumQuickDofs(); 
  PRINT(mLinks.size());
  mLinks.resize(mNumLinks);
  PRINT(mLinks.size());
  int EEDofId = mLinks( mNumLinks -1); // TODO: use [] instead?
  mEEId = mWorld.getRobot(mRobotId)->getDof( EEDofId )->getJoint()->getChildNode()->getSkelIndex();
  std::string mEEName =  mWorld.getRobot(mRobotId)->getNode(mEEId)->getName();
  
  //init goal_finder
  mMaxIter = 1000;
  mWorkspaceThresh = 0.02; // An error of half the resolution // dunno why 0.02
  mEENode = (dynamics::BodyNodeDynamics*)mWorld.getRobot(mRobotId)->getNode(mEEName.c_str());
}

MatrixXd JointMover::GetPseudoInvJac() {
  //printf("Num Dependent DOF minus 6D0F is : %d \n", mEENode->getNumDependentDofs() - 6 );
  MatrixXd Jaclin = mEENode->getJacobianLinear().topRightCorner( 3, mLinks.size() );
  //std::cout<< "Jaclin: \n"<<Jaclin << std::endl;
  MatrixXd JaclinT = Jaclin.transpose();
  MatrixXd Jt;
  MatrixXd JJt = (Jaclin*JaclinT);
  FullPivLU<MatrixXd> lu(JJt);
  Jt = JaclinT*( lu.inverse() );
  //std::cout<< "Jaclin pseudo inverse: \n"<<Jt << std::endl;  
  return Jt;
}

VectorXd JointMover::OneStepTowardsXYZ( VectorXd _q, VectorXd _targetXYZ) {
  ECHO("  BEGIN OneStepTowardsXYZ");
  assert(_targetXYZ.size() == 3);
  assert(_q.size() > 3);
  VectorXd dXYZ = _targetXYZ - GetXYZ(_q); // GetXYZ also updates the config to _q, so Jaclin use an updated value
  VectorXd dConfig = GetPseudoInvJac()*dXYZ;

  double alpha = min((mConfigStep/dConfig.norm()), 1.0); // Constant to not let vector to be larger than mConfigStep
  dConfig = alpha*dConfig;

  ECHO("  END OneStepTowardsXYZ");
  return _q + dConfig;
}


bool JointMover::GoToXYZ( VectorXd &_q, VectorXd _targetXYZ) {
  ECHO("BEGIN GoToXYZ");

  mWorld.getRobot(mRobotId)->update();
  VectorXd dXYZ = _targetXYZ - GetXYZ(_q); // GetXYZ also updates the config to _q, so Jaclin use an updated value
  int iter = 0;
  while( dXYZ.norm() > mWorkspaceThresh && iter < mMaxIter ) {
    _q = OneStepTowardsXYZ(_q, _targetXYZ);
    dXYZ = (_targetXYZ - GetXYZ(_q) );
    iter++;
  }

  ECHO("END GoToXYZ");
  return iter < mMaxIter;
}

VectorXd JointMover::GetXYZ( VectorXd _q ) {
  // Get current XYZ position

  mWorld.getRobot(mRobotId)->setDofs(_q, mLinks );
  mWorld.getRobot(mRobotId)->update();

  MatrixXd qTransform = mEENode->getWorldTransform();
  VectorXd qXYZ(3); qXYZ << qTransform(0,3), qTransform(1,3), qTransform(2,3);

  return qXYZ;
}


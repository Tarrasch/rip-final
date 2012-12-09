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
#include "newtonianPhysics.h"

using namespace std;
using namespace Eigen;

JointMover::JointMover( robotics::World &_world, int _robotId, double _configStep)
  : mConfigStep(_configStep), mWorld(_world), mRobotId(_robotId) {

  mLinks = mWorld.getRobot(mRobotId)->getQuickDofsIndices();
  //get data to initialize
  
  int mNumLinks = mWorld.getRobot(mRobotId)->getNumQuickDofs(); 
  mLinks.resize(mNumLinks);

  int EEDofId = mLinks( mNumLinks -1); // TODO: use [] instead?
  mEEId = mWorld.getRobot(mRobotId)->getDof( EEDofId )->getJoint()->getChildNode()->getSkelIndex();
  std::string mEEName =  mWorld.getRobot(mRobotId)->getNode(mEEId)->getName();
  
  //init goal_finder
  mMaxIter = 1000;
  mWorkspaceThresh = 0.02; // An error of half the resolution // dunno why 0.02
  mEENode = (dynamics::BodyNodeDynamics*)mWorld.getRobot(mRobotId)->getNode(mEEName.c_str());

  // Precalculate pseduojacobian
  //printf("Num Dependent DOF minus 6D0F is : %d \n", mEENode->getNumDependentDofs() - 6 );
  MatrixXd Jaclin = mEENode->getJacobianLinear().topRightCorner( 3, mLinks.size() );
  //std::cout<< "Jaclin: \n"<<Jaclin << std::endl;
  MatrixXd JaclinT = Jaclin.transpose();
  MatrixXd JJt = (Jaclin*JaclinT);
  FullPivLU<MatrixXd> lu(JJt);
  pseudoInvJac = JaclinT*( lu.inverse() );

  //std::cout<< "Jaclin pseudo inverse: \n"<<Jt << std::endl;  
}

MatrixXd JointMover::GetPseudoInvJac() {
  return pseudoInvJac;
}

VectorXd JointMover::OneStepTowardsXYZ( VectorXd _q, VectorXd _targetXYZ) {
  // ECHO("  BEGIN OneStepTowardsXYZ");
  assert(_targetXYZ.size() == 3);
  assert(_q.size() > 3);
  VectorXd dXYZ = _targetXYZ - GetXYZ(_q); // GetXYZ also updates the config to _q, so Jaclin use an updated value
  VectorXd dConfig = GetPseudoInvJac()*dXYZ;

  double alpha = min((mConfigStep/dConfig.norm()), 1.0); // Constant to not let vector to be larger than mConfigStep
  dConfig = alpha*dConfig;

  // ECHO("  END OneStepTowardsXYZ");
  return _q + dConfig;
}


bool JointMover::GoToXYZ( VectorXd _qStart, VectorXd _targetXYZ, VectorXd &_qResult) {
  ECHO("BEGIN GoToXYZ");
  _qResult = _qStart;
  mWorld.getRobot(mRobotId)->update();
  VectorXd dXYZ = _targetXYZ - GetXYZ(_qResult); // GetXYZ also updates the config to _qResult, so Jaclin use an updated value
  int iter = 0;
  while( dXYZ.norm() > mWorkspaceThresh && iter < mMaxIter ) {
    _qResult = OneStepTowardsXYZ(_qResult, _targetXYZ);
    dXYZ = (_targetXYZ - GetXYZ(_qResult) );
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

static double JointMover::jointSpaceDistance(VectorXd _q1, VectorXd _q2) {
  // This is the infinite norm
  return (_q2-_q1).cwiseAbs().maxCoeff();
}

static VectorXd JointMover::jointSpaceMovement(VectorXd _qStart, VectorXd _qGoal) {
  VectorXd diff = (_qGoal-_qStart);
  for(int i = 0; i < diff.size(); i++){
    diff[i] = max(-jointSpeeds*dt, min(jointSpeeds*dt, diff[i]));
  }
  return _qStart + diff;
}

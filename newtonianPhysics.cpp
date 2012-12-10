#include <iostream>
#include <stdlib.h>
#include <Eigen/LU>
#include "newtonianPhysics.h"
#include <time.h>

#define PRINT(x) std::cout << #x << " = " << x << std::endl;
#define ECHO(x) std::cout << x << std::endl;

Path projectileMotion(VectorXd pos, VectorXd vel, VectorXd acc){
        //ECHO("START Projectile Motion");
        Path res;
        do {
                res.push_back(pos);
                pos += vel*dt + acc*pow(dt,2)/2.0;
                vel += acc*dt;
                
        } while (pos[2] > 0);
        //ECHO("DONE Projectile Motion");
        return res;
}


Path projectileMotionWRand(VectorXd pos, VectorXd vel, VectorXd acc, int randMaxAcc){
	//ECHO("START Random Projectile Motion"); 
	
	// initialize random seed
	srand ( time(NULL) );					

	// Build a list of VectorXds for the final path
	Path res;				
	do {
		// Push the current position to the final path list
		res.push_back(pos);
		
		// Build a temporary vector for the final randomized acceleration
		VectorXd tempAcc;	

		// Resize the list to the appropriate length/size
		tempAcc.resize(3);	

		// Generate a random acceleration
		float randNum = (float)rand()/((float)RAND_MAX/(randMaxAcc)) - (float)randMaxAcc/2;
		
		// Add the random acceleration to the temporary randomized acceleration
		tempAcc << randNum + acc[0], randNum + acc[1], randNum + acc[2];
		
		// Generate a new position for the path
		pos += vel*dt + tempAcc*dt*dt/2.0;
		
		// Generate a new velocity for the next point on the path
		vel += tempAcc*dt;
	} while (pos[2] > 0);
	//ECHO("DONE Projectile Motion");
	return res;
}

Path straightMotion(VectorXd startpos, VectorXd endpos){
        //ECHO("START Straight Motion");
        double t = 1;
        VectorXd dir = endpos - startpos;
        double distance = dir.norm();
        VectorXd vel = dir*(1/t);
 
        Path res;
        VectorXd pos = startpos;
        do {
                res.push_back(pos);
                pos += vel*dt;
                
        } while ((pos-startpos).norm() < distance);
        //ECHO("END Straight Motion");
        return res;
}

Path projectileMotionWRandT(VectorXd pos, VectorXd vel, VectorXd acc, int randMaxAcc, double maxTime){
	//ECHO("START Random Projectile Motion with Time"); 
	
	// initialize random seed
	srand ( time(NULL) );					

	// Build a list of VectorXds for the final path
	Path res;		

	// Build a counter to calculate the amount of "time" that has passed
	double t = 0;
	do {
		// Push the current position to the final path list
		res.push_back(pos);
		
		// Build a temporary vector for the final randomized acceleration
		VectorXd tempAcc;	

		// Resize the list to the appropriate length/size
		tempAcc.resize(3);	

		// Generate a random acceleration
		float randNum = (float)rand()/((float)RAND_MAX/(randMaxAcc)) - (float)randMaxAcc/2;
		
		// Add the random acceleration to the temporary randomized acceleration
		tempAcc << randNum + acc[0], randNum + acc[1], randNum + acc[2];
		
		// Generate a new position for the path
		pos += vel*dt + tempAcc*dt*dt/2.0;
		
		// Generate a new velocity for the next point on the path
		vel += tempAcc*dt;
		
		// Increment the time t
		t += dt;
	} while (t < maxTime);
	//ECHO("DONE Projectile Motion (Bounded by time)");
	return res;
}


VectorXd calculateVelocities(VectorXd startpos, VectorXd endpos){
        VectorXd vel(3);
        double t = 1.0;
        VectorXd distance = (endpos - startpos);
        
        /*double d = sqrt(pow(distance,2) + pow(startpos[1],2));
        double v = sqrt((d*pg)/(sin(2*theta)));
        //now find velocity for each axes
        double vx = v * cos(theta);
        double vy = v * sin(theta);
        double vz = v * tan(theta);
        vel << vx,vy,vz;
        PRINT(distance);*/
        vel = distance *(1.0/t);
        //PRINT(vel);
        return vel;
}


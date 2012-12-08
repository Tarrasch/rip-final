#include <iostream>
#include <stdlib.h>
#include <Eigen/LU>
#include "newtonianPhysics.h"

#define PRINT(x) std::cout << #x << " = " << x << std::endl;
#define ECHO(x) std::cout << x << std::endl;

const double g = (-9.82/10.0);
const double pg = -g;
const double PI = 3.1415925;
const double dt = 0.05;

std::list< VectorXd > projectileMotion(VectorXd pos, VectorXd vel, VectorXd acc){
        ECHO("START Projectile Motion");
        std::list<VectorXd> res;
        do {
                res.push_back(pos);
                pos += vel*dt + acc*pow(dt,2)/2.0;
                vel += acc*dt;
                
        } while (pos[2] > 0);
        ECHO("DONE Projectile Motion");
        return res;
}

std::list< VectorXd > straightMotion(VectorXd startpos, VectorXd endpos){
        double t = 1;
        VectorXd dir = endpos - startpos;
        double distance = dir.norm();
        VectorXd vel = dir*(1/t);
        PRINT(vel);
 
        std::list<VectorXd> res;
        VectorXd pos = startpos;
        do {
                res.push_back(pos);
                pos += vel*dt;
                
        } while ((pos-startpos).norm() < distance);
        return res;
}

VectorXd calculateVelocities(VectorXd startpos, VectorXd endpos){
        VectorXd vel(3);
        
        //acceleration vector
        VectorXd acc;
        acc.resize(3);
        acc << 0, 0, g;
       
        double distance = (startpos - endpos).norm();
        PRINT(distance);
        
        PRINT(sqrt(distance*pg));
        
        double v = sqrt(distance*pg);
        //now find velocity for each axes; need projectile angle
        double angle = asin((pg*distance)/pow(v,2));
        angle = (angle * 180.0 / PI) / 2.0 ;
        PRINT(angle);
        
        double vx = v * cos(angle);
        double vy = v * sin(angle);
        
        vel << vx, 0, vy;
        return vel;
}


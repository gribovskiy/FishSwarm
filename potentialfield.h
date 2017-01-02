//Autor : Laila El Hamamsy
//Date Created : Tuesday November 15th 2016
//Version : 3
//Last Modified :


#ifndef POTENTIALFIELD_H
#define POTENTIALFIELD_H

#include <QPoint>
#include <QDebug>

#include <vector>
#include <stdbool.h>
#include <math.h>


#include "constants.h"
#include "fishrobot.h"
#include "target.h"
#include "rectangle.h"

class FishRobot;

//! Computes the values of the forces at each x,y coordinate in the arena for each
//! fishRobot with respect to each target.

class PotentialField
{
public:
    //! this constructor takes in the configuration space as well as the
    //! vector of fishRobots in the simulation
    PotentialField(std::vector<std::vector<enum State>> configurationSpace,
                   std::vector<FishRobot*> *fishRobots,
                   enum Approach  potfield = Approach::LOCAL);

    //! Compute the total force on a robot, both attractive and repulsive
    //! this given a fishRobot id and a target
    std::pair<float,float> computeTotalForceForRobot(int fishRobotId, QPoint targetPos);

    //! this method updates the parameters of the potential field
    void setParameters(int newNuRobots, int newRho0Robots, int newNuArena, int newRho0Arena,
                       int newZeta, int newdGoalStar, int newMaxForce, int newMaxAngle);

private:

    //! Contains the discretized configuration space
    std::vector<std::vector<State>> m_configurationSpace;
    //! Contains the values of the repulsive forces at each x,y coordinate due to obstacles
    std::vector<std::vector<std::pair<float,float>>> m_globalRepulsive;
    //! To access the position, angle of all fishRobots and their targets
    std::vector<FishRobot*>* m_fishRobots;
    //! configuration space width and height
    int m_width, m_height;
    //! Approach used
    enum Approach m_approach;

    //-------------------------------------------//
    //----Setting up the Configuration Space-----//
    //-------------------------------------------//
    //! Discretizes the newConfiguration Space
    void setNewConfigurationSpace(std::vector<std::vector<State>> newConfigurationSpace);
    //! Determines whether the cell is free or occupied
    State getCellState(std::vector<std::vector<State>> newConfigurationSpace,
                                    int column,int row,int m_step);
    //! Resets the global repulsive force to 0
    void reinitializeConfigSpaceRepulsiveForces();
    //! Identify the obstacles' borders in the configuration space
    void identifyConfigurationSpaceBorders(std::vector<QPoint>* obstaclesBorders);

    //-------------------------------------------//
    //------Computing Repulsive Forces-----------//
    //-------------------------------------------//

    //! Method to compute all repulsive forces
    std::pair<float,float> computeAllRepulsiveForces(int fishRobotId);
    //! Computes the repulsive force due to the arena in the configuration space
    std::pair<float,float> computeLocalRepulsiveForceDueToArena(int fishRobotId);
    //! Computes the repulsive force due to the obstacles in the configuration space
    void                   computeGlobalRepulsiveForceDueToArena();
    //! Computes the local repulsive force due to other robots for a specific robot
    std::pair<float,float> computeRepulsiveForceDueToRobots(int fishRobotId);
    //! Comptues the local repulsive force
    std::pair<float,float> computeLocalRepulsiveForce(QPoint currentPos, QPoint obstaclePos);

    //-------------------------------------------//
    //------Computing Attractive Forces----------//
    //-------------------------------------------//
    //! Computes the local attractive force due to target for a specific robot
    std::pair<float,float> computeAttractiveForce(int fishRobotId, QPoint targetPos);

    //-------------------------------------------//
    //---------Potential Field Tuning------------//
    //-------------------------------------------//

    //! Repulsive Parameters rho0 being the distance of influence and nu the "strength" of the repulsion
    float m_rho0Arena,  m_nuArena;
    float m_rho0Robots, m_nuRobots;
    float m_nu,  m_rho0;
    //! Attractive Parameters dgoalstar being the distance of influence and zeta the "strength" of the attraction
    float m_zeta;
    float m_dGoalStar;
    //! General Parameters
    float m_maxForce;
    float m_maxAngle;
    //! Potential Field Map Parameters
    int m_distCell;
};

#endif // POTENTIALFIELD_H

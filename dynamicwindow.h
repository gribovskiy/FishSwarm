//Autor : Laila El Hamamsy
//Date Created : Sunday December 4th 2016
//Version : 1
//Last Modified : 26.12.2016

#ifndef DYNAMICWINDOW_H
#define DYNAMICWINDOW_H

#include <stdbool.h>
#include <math.h>
#include <vector>
#include <QDebug>

#include "constants.h"
#include "fishrobot.h"
#include "target.h"
#include "priorityplanning.h"

//! FIXME : get correct values
#define DIST_WHEELS  16 //distance between the wheels 2cm

const int   linearAcceleration = 40; //en px/sec (considered to be the same along x and y)
const float angularAcceleration = linearAcceleration/DIST_WHEELS;
const int   numberLinearVel = 10;
const int   numberAngularVel = 10;
const int   dtSamples = 100; //! equivalent to 3 seconds


struct COLLISIONDIST
{
    float distance = 0;
    bool collision = false;

    COLLISIONDIST(bool collides, float dist)
    {
        collision = collides;
        distance = dist;
    }
};

class FishRobot;
class PriorityPlanning;

class DynamicWindow
{
public:
    //! this constructor takes in the configuration space as well as the
    //! vector of fishRobots in the simulation
    DynamicWindow(std::vector<std::vector<enum State>> configurationSpace,
                  std::vector<FishRobot*> *fishRobots);

    //! Compute the new linear and angular velocities for the given robot if an obstacle is in the vicinity
    //! if no obstacle is detected it returns -1,-1
    std::pair<float,float> computeNewLinearAndAngularVelIfObstacle(int fishRobotId, QPoint pathGoal);

    //! this method updates the parameters of the dynamic window
    void setParameters(int newAlpha, int newBeta, int newGamma, int newDelta, float newTimeframe = dtSamples*simulation_dt);

private:

    //! Contains the configuration space
    std::vector<std::vector<State>> m_configurationSpace;
    //! Contains the robots space
    std::vector<std::vector<State>> m_robotsSpace;
    //! To access the position, angle of all fishRobots and their targets
    std::vector<FishRobot*>* m_fishRobots;
    //! Priority planning to avoid the situation where the robots no longer can move
    PriorityPlanning *m_priorityPlanning;
    //! configuration space width and height
    int m_width, m_height;
    //! current fish robot id
    int m_fishRobotId;
    //! current fish robot velocity
    int m_vel;
    int m_angularVel;
    //! maximum admissible fish robot velocity
    int m_maxVel;
    int m_maxAngularVel;
    //! intermediate target position
    QPoint m_goal;
    //! current robot position
    QPoint m_pos;
    //! current robot angle
    float m_angle;
    //! list of admissible velocities
    std::vector<std::tuple<float,float,float>> m_admissibleVelocities;

    //-------------------------------------------//
    //----------Non Exported Methods-------------//
    //-------------------------------------------//

    //! this method positions the fish robot in their own configuration space to
    //! take into account the robot dimensions for the obstacle avoidance.
    void initializeRobotSpace();
    //! this method identifies whether or not there is an obstacle on the current
    //! path
    bool robotCloseby(QPoint currentPos, int distRatio = 1);

    //! this method goes through the search space to identify all admissible
    //! velocities, ie the ones that do not lead to collision within a certain timeframe

    void searchSpaceForAdmissibleVelocities();

    //! this method computes the square of the distance the robot will travel given a current
    //! timeframe and a circular trajectory

    COLLISIONDIST distanceTravelled(int v, int omega);

    //! this method chooses the most optimal linear and angular velocities by using
    //! an objective function and choosing the highest scoring combination
    std::pair<float,float> identifyOptimalVelocities();

    //! this method computes the total objective function for a given linear and angular
    //! velocity
    float computeObjectiveFunction(std::tuple<float,float,float> velocity);

    //! this method computes the align objective function for a given localization
    //! velocity to insure the robot stays directed towards the goal
    float computeAlignFunction(QPointF newPos,float newAngle);

    //! this method computes the velocity objective function for a given linear and angular
    //! velocity to insure the robot prefers fastforward motion
    float computeVelocityFunction(std::tuple<float,float,float> velocity);

    //! this method computes the goal objective function for a given position
    //! velocity to insure the robot keeps going towards the goal
    int computeGoalFunction(QPointF newPos);

    //! this method computes the distance objective function for a given localization
    //! and velocity. It returns the longest distance the robot can go without
    //! collision with respect to the total distance it can travel within the given timeframe
    float computeDistanceFunction(std::tuple<float,float,float> velocity);

    //-------------------------------------------//
    //----------Dynamic Window Tuning------------//
    //-------------------------------------------//

    //! Alignment parameter tuning
    int m_alpha;
    //! Velocity parameter tuning
    int m_beta;
    //! Goal region parameter tuning
    int m_gamma;
    //! Distance parameter tuning
    int m_delta;
    //! Timeframe parameter tuning
    float m_timeframe;
};

#endif // DYNAMICWINDOW_H

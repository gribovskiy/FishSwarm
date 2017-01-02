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
#include <QVector2D>
#include <QRect>

#include "constants.h"
#include "fishrobot.h"
#include "target.h"
#include "priorityplanning.h"
#include "rectangle.h"


#define DIST_WHEELS  16 //distance between the wheels 2cm

const int   linearAcceleration = 40; //en px/sec2 (considered to be the same along x and y)
const float angularAcceleration = linearAcceleration/DIST_WHEELS;
const int   numberLinearVel = 10;
const int   numberAngularVel = 15;
const int   m_step = 1;
const int   dtSamples = 60/m_step; //! equivalent to 3 seconds


//! this structure stores the distance travelled and whether or not a collision
//! has been detected
struct COLLISIONDIST
{
    float distance;
    bool collision;

    COLLISIONDIST(bool collides = false, float dist = 0)
    {
        collision = collides;
        distance = dist;
    }
};

//! this structure stores the tuples v and omega as well as the distance
//! that can be travelled without collision within the given timeframe. It also
//! stored whether a collision has been detected.
struct VELOCITIESDIST
{
    float           linearVel;
    float           angularVel;
    COLLISIONDIST   collisionDistance;

    VELOCITIESDIST(float linVel, float angVel, COLLISIONDIST collisionDist)
    {
        collisionDistance.collision = collisionDist.collision;
        collisionDistance.distance = collisionDist.distance;
        linearVel = linVel;
        angularVel = angVel;
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

    //! Compute the new linear and angular velocities for the given robot if an
    //! obstacle is in the vicinity if no obstacle is detected it returns -1,-1
    //! in float, needs to be rounded to int for testing
    std::pair<float,float> computeNewLinearAndAngularVelIfObstacle(int fishRobotId,
                                                                   QPoint pathGoal);

    //! this method updates the parameters of the objective function of the dynamic window
    void setObjectiveFunctionParameters(int newAlpha, int newBeta, int newGamma,
                                        int newDelta, float newDistLimitGoal = 30,
                                        float newTimeframe = dtSamples*simulation_dt);

    //! this method updates the parameters of the defined occupied zones of the dynamic window
    void setOccupiedZoneParameters(float newDistLimitRobot, float newAngleLimitRAD);

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
    std::vector<VELOCITIESDIST> m_admissibleVelocities;
    //! list of IDs to ignore in case of required priority planning
    std::vector<int> m_ignoreIDs;

    //-------------------------------------------//
    //----------Non Exported Methods-------------//
    //-------------------------------------------//

    //---------------------------------//
    //----------Initialization---------//
    //---------------------------------//

    //! Non Exported Member. This method initializes the parameters for the dynamic
    //! window for the given robot
    void initializeParametersRobot (int fishRobotId, QPoint pathGoal);

    //---------------------------------//
    //----Occupancy and Collision------//
    //---------------------------------//

    //! this method positions the fish robot in their own configuration space to
    //! take into account the robot dimensions for the obstacle avoidance.
    void initializeRobotSpace();

    //! this method identifies which robots are close to the defined position
    std::vector<int> identifyRobotsCloseBy(QPoint pos);

    //! this method identifies whether or not the given robot is close to the given
    //! position
    bool robotCloseby(QPoint currentPos, int robotID);

    //! this method determines whether all the robots given are blocked as defined
    //! by their status in the fishrobot.cpp class
    bool allRobotsBlocked(std::vector<int> robotsCloseBy);

    //! this method computes the square of the distance the robot will travel given a current
    //! timeframe and a circular trajectory
    COLLISIONDIST distanceTravelled(int v, int omega);

    //! This method detects whether there is a collision betweem the current
    //! robot and the others. The position
    bool collisionWithOtherRobots(QPoint pos, float orientation);

    //! This method detects whether there is a collision betweem the current
    //! robot and the others when the position obtained is in the form of a QPointF
    //! It converts the QPointF to QPoint then calls the original collision check
    //! function
    bool collisionWithOtherRobots(QPointF pos, float orientation);

    //! this method tests whether the current index is in the list of indices to be
    //! ignored for priority planning
    bool ignoredID(int index);

    //---------------------------------//
    //---Dyanmic Window Computation----//
    //---------------------------------//

    //! this method goes through the search space to identify all admissible
    //! velocities, ie the ones that do not lead to collision within a certain timeframe
    void searchSpaceForAdmissibleVelocities();


    //! this method chooses the most optimal linear and angular velocities by using
    //! an objective function and choosing the highest scoring combination
    std::pair<float,float> identifyOptimalVelocities();


    //---------------------------------//
    //-Objective Function Computation--//
    //---------------------------------//

    //! this method computes the total objective function for a given linear and angular
    //! velocity
    float computeObjectiveFunction(VELOCITIESDIST velocity);

    //! this method computes the align objective function for a given localization
    //! velocity to insure the robot stays directed towards the goal
    float computeAlignFunction(QPointF newPos,float newAngle);

    //! this method computes the velocity objective function for a given linear and angular
    //! velocity to insure the robot prefers fastforward motion
    float computeVelocityFunction(VELOCITIESDIST velocity);

    //! this method computes the goal objective function for a given position
    //! velocity to insure the robot keeps going towards the goal
    int computeGoalFunction(QPointF newPos);

    //! this method computes the distance objective function for a given localization
    //! and velocity. It returns the longest distance the robot can go without
    //! collision with respect to the total distance it can travel within the given timeframe
    float computeDistanceFunction(VELOCITIESDIST velocity);

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
    //! Limit distance to goal parameter tunning
    float m_distLimitGoal;
    //! Limit distance to robots parameter tunning
    float m_distLimitRobot;
    //! Limit of perception tuning
    float m_angleLimit;

};

#endif // DYNAMICWINDOW_H

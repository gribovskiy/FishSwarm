//Autor : Laila El Hamamsy
//Date Created : Sunday December 4th 2016
//Version : 1
//Last Modified :


#ifndef DYNAMICWINDOW_H
#define DYNAMICWINDOW_H

#include <stdbool.h>
#include <math.h>
#include <vector>

#include "constants.h"
#include "fishrobot.h"
#include "lures.h"

class FishRobot;

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
    void setParameters(int newAlpha, int newBeta, int newGamma, float newTime);

private:

    //! Contains the configuration space
    std::vector<std::vector<State>> m_configurationSpace;
    //! To access the position, angle of all fishRobots and their targets
    std::vector<FishRobot*>* m_fishRobots;
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
    std::vector<std::pair<float,float>> m_admissibleVelocities;

    //-------------------------------------------//
    //----------Non Exported Methods-------------//
    //-------------------------------------------//

    //! this method identifies whether or not there is an obstacle on the current
    //! path
    bool robotCloseby(QPoint currentPos);

    //! this method goes through the search space to identify all admissible
    //! velocities, ie the ones that do not lead to collision within a certain timeframe

    void searchSpaceForAdmissibleVelocities();

    //! this method computes the distance the robot will travel given a current
    //! timeframe and a circular trajectory

    float dist(int v, int omega);

    //! this method chooses the most optimal linear and angular velocities by using
    //! an objective function and choosing the highest scoring combination
    std::pair<float,float> identifyOptimalVelocities();

    //! this method computes the total objective function for a given linear and angular
    //! velocity
    float computeObjectiveFunction(std::pair<float,float> velocity);

    //! this method computes the align objective function for a given linear and angular
    //! velocity to insure the robot stays directed towards the goal
    float computeAlignFunction();

    //! this method computes the velocity objective function for a given linear and angular
    //! velocity to insure the robot prefers fastforward motion
    float computeVelocityFunction(std::pair<float,float> velocity);

    //! this method computes the goal objective function for a given linear and angular
    //! velocity to insure the robot keeps going towards the goal
    int computeGoalFunction();

    //-------------------------------------------//
    //----------Dynamic Window Tuning------------//
    //-------------------------------------------//

    //! Alignment parameter tuning
    int m_alpha;
    //! Velocity parameter tuning
    int m_beta;
    //! Goal region parameter tuning
    int m_gamma;
    //! Timeframe parameter tuning
    float m_timeframe;
};

#endif // DYNAMICWINDOW_H

#ifndef POTENTIALFIELD_H
#define POTENTIALFIELD_H

#include <QPoint>

#include <vector>
#include <stdbool.h>
#include <math.h>


#include "constants.h"
#include "fishrobot.h"
#include "lures.h"


// FIXME : need a comment for every method (and for every class), for instance:

//! Computes the values of the forces at each x,y coordinate in the arena for each
//! fishRobot with respect to each target.

class PotentialField
{
public:
    //! this constructor takes in the configuration space as an input in order to
    //construct the global repulsive potential field
    PotentialField(std::vector<std::vector<enum State>> configurationSpace,
                   std::vector<FishRobot*> fishRobots);

    //! Compute the total force on a robot, both attractive and repulsive
    std::pair<float,float> computeTotalForceForRobot(int fishRobotId);

private:

    //! Contains the values of the repulsive forces at each x,y coordinate due to obstacles
    std::vector<std::vector<std::pair<float,float>>> m_globalRepulsive;
    //! To access the position, angle of all fishRobots and their targets
    std::vector<FishRobot*> m_fishRobots;
    //! configuration space width and height
    int m_width, m_height;



    //! Computes the global repulsive force due to the obstacles in the configuration space
    void computeConfigSpaceRepulsiveForce(std::vector<std::vector<enum State>> configurationSpace);
    //! Identify the obstacles' borders in the configuration space
    void identifyConfigurationSpaceBorders(std::vector<QPoint>* obstaclesBorders,
                                           std::vector<std::vector<enum State>> configurationSpace);
    //! Resets the global repulsive force to 0
    void reinitializeConfigSpaceRepulsiveForces();
    //! Computes the local repulsive force due to other robots for a specific robot
    std::pair<float,float> computeLocalRobotsRepulsiveForce(int fishRobotId);
    //! Computes the local attractive force due to target for a specific robot
    std::pair<float,float> computeLocalAttractiveForce(int fishRobotId);
    //! Comptues the local repulsive force
    std::pair<float,float> computeLocalRepulsiveForce(QPoint currentPos, QPoint obstaclePos);


};

#endif // POTENTIALFIELD_H

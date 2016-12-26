//Autor : Laila El Hamamsy
//Date Created : Monday December 26th 2016
//Version : 1
//Last Modified :

#ifndef PRIORITYPLANNING_H
#define PRIORITYPLANNING_H

#include <stdbool.h>
#include <math.h>
#include <vector>
#include <QDebug>


#include "constants.h"
#include "fishrobot.h"
#include "lures.h"


class FishRobot;

//! this class will help determine in the case of blocage which robot has
//! priority over the others.
class PriorityPlanning
{
public:
    //-------------------------------------------//
    //--------------Exported Methods-------------//
    //-------------------------------------------//

    //! class constructor, it requires access to all the fish robots in the simulation
    //! as well as the djikstra path to determine which one is closest or farthest
    //! to its target depending on the chosen strategy
    PriorityPlanning(std::vector<FishRobot*> *fishRobots, float maxDist,
                     Strategy strategy = Strategy::CLOSEST);

    //! this method returns the ID of the fish robot that is either closest or farthest
    //! from its final target given the djikstra shortest path depending on the chosen
    //! strategy
    int getOptimalFishRobotID();


    //! this method changes the strategy for the priority planning
    void setNewStrategy(Strategy newStrategy);


private:

    //! stores the chosen strategy for the priority planning
    Strategy m_strategy = Strategy::CLOSEST;

    //! To access the position, angle of all fishRobots and their targets
    std::vector<FishRobot*>* m_fishRobots;

    //! stores the maximum distance that can be travelled within the configuration space
    float m_maxDist;

    //-------------------------------------------//
    //----------Non Exported Methods-------------//
    //-------------------------------------------//

    //! this method returns the ID of the fishRobot that is closest to its final
    //! target
    int getClosestFishRobotID();

    //! this method returns the ID of the fishRobot that is farthest from its final
    //! target
    int getFarthestFishRobotID();

    //! this method returns the distance to the final target given the dijkstra path
    float getDijkstraDistance(int fishRobotID);

    //! this method computes the distance between 2 points.
    float computeDistance(QPoint pos1, QPoint pos2);

};

#endif // PRIORITYPLANNING_H

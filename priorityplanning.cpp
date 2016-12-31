//Autor : Laila El Hamamsy
//Date Created : Monday December 26th 2016
//Version : 1
//Last Modified : 26.12.2016


#include "priorityplanning.h"

//----------------------------------------------------------------------------//
//-------------------------------Class Constructors---------------------------//
//----------------------------------------------------------------------------//

//! class constructor, it requires access to all the fish robots in the simulation
//! as well as the djikstra path to determine which one is closest or farthest
//! to its target depending on the chosen strategy

PriorityPlanning::PriorityPlanning(std::vector<FishRobot*> *fishRobots,float maxDist,
                                   Strategy strategy)
{
    //! Store fishRobots and Target pointers
    m_fishRobots = fishRobots;

    //! Update strategy
    m_strategy = strategy;

    //! store the maximum distance that can be travelled within the config. space
    m_maxDist = maxDist;
}

//----------------------------------------------------------------------------//
//--------------------------------Exported Members----------------------------//
//----------------------------------------------------------------------------//

//! this method returns the ID of the fish robot that is either closest or farthest
//! from its final target given the djikstra shortest path depending on the chosen
//! strategy
int PriorityPlanning::getOptimalFishRobotID(std::vector<int>blockedRobotsID)
{
    int OptimalFishRobotID = -1;

    //! if the strategy is closest, get the ID of the closest Fish Robot
    if (m_strategy == Strategy::CLOSEST)
    {
        OptimalFishRobotID = getClosestFishRobotID(blockedRobotsID);
    }
    //! else if the strategy is farthest, get the ID of the farthest Fish Robot
    else if (m_strategy == Strategy::FARTHEST)
    {
        OptimalFishRobotID = getFarthestFishRobotID(blockedRobotsID);
    }

    return OptimalFishRobotID;
}

//-------------------------------------------//
//-------------Setter Functions--------------//
//-------------------------------------------//


//! this method changes the strategy for the priority planning
void PriorityPlanning::setNewStrategy(Strategy newStrategy)
{
    m_strategy = newStrategy;
}


//----------------------------------------------------------------------------//
//------------------------------Non Exported Members--------------------------//
//----------------------------------------------------------------------------//

//! this method identifies the ID of the fishRobot that is closest to its final
//! target
int PriorityPlanning::getClosestFishRobotID(std::vector<int>blockedRobotsID)
{
    float closestDist = m_maxDist;
    int closestID = -1;
    float dist;

    //! for all the fishBots
    for (int i = 0 ; i< (int)blockedRobotsID.size(); i++)
    {
        if(blockedRobotsID.at(i)<(int)m_fishRobots->size())
        {
            //! compute the distance to the final target for the considered blocked robot
            dist = getDijkstraDistance(blockedRobotsID.at(i));
            //! if the distance is inferior to the closest distance
            if (dist<closestDist)
            {
                //! update closest distance and closest ID
                closestDist = dist;
                closestID = blockedRobotsID.at(i);
            }
        }
    }

    return closestID;
}

//! this method identifies the ID of the fishRobot that is farthest from its final
//! target
int PriorityPlanning::getFarthestFishRobotID(std::vector<int>blockedRobotsID)
{
    float farthestDist = 0;
    int farthestID = -1;
    float dist;

    //! for all the fishBots
    for (int i = 0 ; i< (int)blockedRobotsID.size(); i++)
    {
        if(blockedRobotsID.at(i)<(int)m_fishRobots->size())
        {
            //! compute the distance to the final target
            dist = getDijkstraDistance(blockedRobotsID.at(i));

            //! if the distance is superior to the closest distance
            if (dist>farthestDist)
            {
                //! update farthest distance and farthest ID
                farthestDist = dist;
                farthestID = blockedRobotsID.at(i);
            }
        }
    }

    return farthestID;
}

//! this method computes the distance to the final target given the dijkstra path
float PriorityPlanning::getDijkstraDistance(int fishRobotID)
{
    std::vector<QPoint> path = m_fishRobots->at(fishRobotID)->getDijkstraPath();
    float distance = 0;
    QPoint prevPos, currentPos;

    //! if the path is not empty
    if(!path.empty())
    {
        //! the previous position is the position of the fishBot
        prevPos = m_fishRobots->at(fishRobotID)->getPosition();
        //! the current position is the first position in the path
        currentPos = path.at(0);

        //! sum the distance between the two
        distance += computeDistance(prevPos, currentPos);

        //! if there are more path points, go through the vector
        for (int i = 1; i<(int)path.size(); i++)
        {
            //! identify the new current and previous positions
            prevPos = path.at(i-1);
            currentPos = path.at(i);
            //! sum the distance
            distance += computeDistance(prevPos, currentPos);
        }
    }

    return distance;
}

//! this method computes the distance between 2 points.
float PriorityPlanning::computeDistance(QPoint pos1, QPoint pos2)
{
    float dX = pos1.x()-pos2.x();
    float dY = pos1.y()-pos2.y();

    return sqrt(dX*dX + dY*dY);
}

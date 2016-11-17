#include "potentialfield.h"


const float zeta = 1;
const float nu = 1;
const float dGoalStar = 1;
const float rho0 = 1;
const float maxForce = 1000;

//----------------------------------------------------------------------------//
//-------------------------------Class Constructors---------------------------//
//----------------------------------------------------------------------------//

PotentialField::PotentialField(std::vector<std::vector<enum State>> configurationSpace,
                               std::vector<FishRobot*> fishRobots)
{
    //! intialize width and height parameters
    m_width = configurationSpace.size();

    if (!configurationSpace.at(0).empty())
    {
        m_height = configurationSpace.at(0).size();
    }
    else m_height = 0;

    //! Store fishRobots and Target pointers
    m_fishRobots = fishRobots;

    //! Compute new global repulsive force
    computeConfigSpaceRepulsiveForce(configurationSpace);
}

//----------------------------------------------------------------------------//
//--------------------------------Exported Members----------------------------//
//----------------------------------------------------------------------------//

//! Compute the total force on a robot, both attractive and repulsive
std::pair<float,float> PotentialField::computeTotalForceForRobot(int fishRobotId)
{
    std::pair<float,float> totalForce;
    std::pair<float,float> totalRepulsiveForce;
    std::pair<float,float> totalAttractiveForce;

    QPoint currentPos = m_fishRobots.at(fishRobotId)->getPosition();

    //compute the total repulsive force as the sum of the force due to the robots
    totalRepulsiveForce  = computeLocalRobotsRepulsiveForce(fishRobotId);
    // and the force due to the obstacles in the configuration space

    //! TODO : prendre plutot la moyenne des points autour
    totalRepulsiveForce.first  += m_globalRepulsive.at(currentPos.x()).at(currentPos.y()).first;
    totalRepulsiveForce.second += m_globalRepulsive.at(currentPos.x()).at(currentPos.y()).second;

    //the total attractive force at the current position
    totalAttractiveForce = computeLocalAttractiveForce(fishRobotId);

    //the total force is given by the total repulsive and attractive forces
    totalForce.first = totalRepulsiveForce.first + totalAttractiveForce.first;
    totalForce.second = totalRepulsiveForce.second + totalAttractiveForce.second;
}


//----------------------------------------------------------------------------//
//------------------------------Non Exported Members--------------------------//
//----------------------------------------------------------------------------//


//! compute the global repulsive force for all robots
void PotentialField::computeConfigSpaceRepulsiveForce(std::vector<std::vector<enum State>> configurationSpace)
{
    int i, j, k;
    std::vector<QPoint> obstaclesBorders;
    std::pair<float,float> force;
    QPoint currentPos(0,0);

    //! Reset global repulsive force
    reinitializeConfigSpaceRepulsiveForces();

    //! Identify the obstacles' borders in the configuration space
    identifyConfigurationSpaceBorders(&obstaclesBorders, configurationSpace);

    //! compute the global repulsive function

    //! Go through the image
    for (i = 0 ; i<m_width ; i++)
    {
        for(j =0 ; j<m_height ; j++)
        {
            //! reinitialize local force
            force.first  = 0;
            force.second = 0;

            //! update current position
            currentPos.setX(i);
            currentPos.setY(j);

            //! At each free position compute the repulsive force for all border obstacles
            for (k = 0 ; k<obstaclesBorders.size(); k++)
            {
                QPoint obstaclePos = obstaclesBorders.at(k);
                force = computeLocalRepulsiveForce(currentPos, obstaclePos);

                float forceNorm = sqrt(pow(force.first,2) +pow(force.second,2));

                if (abs(forceNorm)>maxForce)
                {
                    force.first = force.first*maxForce/forceNorm;
                    force.second = force.second*maxForce/forceNorm;
                }

                //! Sum the results
                m_globalRepulsive.at(i).at(j).first  += force.first;
                m_globalRepulsive.at(i).at(j).second += force.second;
            }
        }
    }
}

//! compute the repulsive force around a robot due to the other robots
std::pair<float,float> PotentialField::computeLocalRobotsRepulsiveForce(int fishRobotId)
{
    std::pair<float,float> robotsRepulsiveForce(0,0), force(0,0);
    QPoint currentPos = m_fishRobots.at(fishRobotId)->getPosition();
    QPoint obstaclePos(0,0);
    float  angle;
    int i;

    for (i = 0 ; i<m_fishRobots.size() ; i++)
    {
        if (i != fishRobotId)
        {
            obstaclePos = m_fishRobots.at(i)->getPosition();
            //FIXME : compute angle =

            //if the obstacle is in the robot's path
            if (fabs(angle) < 90)
            {
                //! compute the resulting force
                force = computeLocalRepulsiveForce(currentPos, obstaclePos);

                //! and sum it to the local repulsive force
                robotsRepulsiveForce.first  += force.first;
                robotsRepulsiveForce.second += force.second;
            }
        }
    }

    return robotsRepulsiveForce;
}

//! compute the attractive force by the target on it's robot
std::pair<float,float> PotentialField::computeLocalAttractiveForce(int fishRobotId)
{
    std::pair<float,float> attractiveForce(0,0);
    QPoint currentPos, targetPos, deltaPos;
    float dGoal ;

    currentPos = m_fishRobots.at(fishRobotId)->getPosition();
    targetPos  = m_fishRobots.at(fishRobotId)->getTargetPosition();

    deltaPos = currentPos - targetPos;

    dGoal = sqrt(pow(deltaPos.x(), 2) + pow(deltaPos.y(), 2));


    if (dGoal <= dGoalStar)
    {
        attractiveForce.first  = -zeta*deltaPos.x();
        attractiveForce.second = -zeta*deltaPos.y();
    }
    else
    {
        attractiveForce.first  = - dGoalStar*zeta*deltaPos.x()/dGoal;
        attractiveForce.second = - dGoalStar*zeta*deltaPos.y()/dGoal;
    }

    return attractiveForce;
}

//! compute the local repulsive force at a point due to an obstacle
std::pair<float,float> PotentialField::computeLocalRepulsiveForce(QPoint currentPos, QPoint obstaclePos)
{
    std::pair<float,float> localRepulsiveForce(0,0);
    QPoint deltaPos;
    float   rhoObst;

    deltaPos = currentPos - obstaclePos;

    rhoObst = sqrt(pow(deltaPos.x(), 2) + pow(deltaPos.y(), 2));

    if (rhoObst < rho0)
    {
        localRepulsiveForce.first  = nu*(1/rhoObst - 1/rho0)*deltaPos.x()/pow(rhoObst,3) ;
        localRepulsiveForce.second = nu*(1/rhoObst - 1/rho0)*deltaPos.y()/pow(rhoObst,3) ;
    }
    else
    {
        localRepulsiveForce.first  = 0 ;
        localRepulsiveForce.second = 0 ;
    }

    return localRepulsiveForce;
}

//! If the configuration space is changed, reinitialize all forces,
//! global and local to 0
void PotentialField::reinitializeConfigSpaceRepulsiveForces()
{
    int i, j;

    m_globalRepulsive.clear();

    //! add all the elements and set them all to 0
    for (i = 0 ; i<m_width ; i++)
    {
        std::vector<std::pair<float,float>> row; // Create an empty row
        for (j = 0; j<m_height ;j++)
        {
            row.push_back(std::make_pair(0,0));
        }
        m_globalRepulsive.push_back(row);
    }
}


//! Identify the obstacles' borders in the configuration space
void PotentialField::identifyConfigurationSpaceBorders(std::vector<QPoint>* obstaclesBorders,
                                                       std::vector<std::vector<enum State>> configurationSpace)
{
    State nodeState;

    int i, j, k, l;
    bool border = false;

    for (i = 0 ; i<m_width ; i++)
    {
        for (j = 0; j<m_height ;j++)
        {
            if (configurationSpace.at(i).at(j) == State::OCCUPIED)
            {
                //! Check the surrounding cells
                for (k = i-1 ; k<=i+1 ; k++)
                {
                    for (l = j-1 ; l<= j+1 ; j++)
                    {
                        //! if the cells are in bounds
                        if (k>=0 && l>=0 && k<m_width && l<m_height)
                        {
                            //if one of these cells is FREE or HALLWAY
                            if (configurationSpace.at(k).at(l) != State::OCCUPIED)
                            {
                                border = true;
                            }
                        }
                    }
                }
                //! If the cell has been defined as a border
                if (border)
                {
                    //! add it to the border list
                    QPoint borderPoint(i,j);
                    obstaclesBorders->push_back(borderPoint);
                }
                //! reset the border status
                border = false;
            }
        }
    }
}

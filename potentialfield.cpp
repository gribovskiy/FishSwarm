//Autor : Laila El Hamamsy
//Date Created : Tuesday November 15th 2016
//Version : 3
//Last Modified :

#include "potentialfield.h"

const int numCells       = 100;

const int distInfluence  = 150;


//----------------------------------------------------------------------------//
//-------------------------------Class Constructors---------------------------//
//----------------------------------------------------------------------------//

PotentialField::PotentialField(std::vector<std::vector<enum State>> configurationSpace,
                               std::vector<FishRobot*>* fishRobots,
                               enum Approach  potfield):m_zeta(2), m_dGoalStar(50),
    m_nuArena(300), m_rho0Arena(100),
    m_nuRobots(1000),m_rho0Robots(30),
    m_maxForce(1000),m_maxAngle(60)
{
    //! Store fishRobots and Target pointers
    m_fishRobots = fishRobots;
    //! Store the approach chosen, local or global
    m_approach = potfield;

    //! intialize parameters with respect to the chosen approach
    if (m_approach == Approach::LOCAL)
    {
        //! store the configuration spaec
        m_configurationSpace = configurationSpace;

        //! compute new width and height
        m_width = configurationSpace.size();

        if(configurationSpace.empty())
        {
            m_height = 0;
        }
        else
        {
            m_height = configurationSpace.at(0).size();
        }
        m_distCell = 1;
    }
    else
    {
        //! set up the new discretized configuration space (to reduce computation time)
        setNewConfigurationSpace(configurationSpace);

        //! Compute new global repulsive force
        computeGlobalRepulsiveForceDueToArena();
    }
}

//----------------------------------------------------------------------------//
//--------------------------------Exported Members----------------------------//
//----------------------------------------------------------------------------//

//! Compute the total force on a robot, both attractive and repulsive
//! this given a fishRobot id and a target
std::pair<float,float> PotentialField::computeTotalForceForRobot(int fishRobotId, QPoint targetPos)
{
    std::pair<float,float> totalForce;
    std::pair<float,float> attractiveForce;
    float forceNorm;

    //! the total attractive force at the current position
    attractiveForce = computeAttractiveForce(fishRobotId, targetPos);

    //! the total repulsive force
    std::pair<float,float> repulsiveForce = computeAllRepulsiveForces(fishRobotId);

    //! the total force is given by the total repulsive and attractive forces
    totalForce.first  = attractiveForce.first + repulsiveForce.first;
    totalForce.second = attractiveForce.second + repulsiveForce.second;

    //! Compute the norm
    forceNorm = sqrt(totalForce.first*totalForce.first+totalForce.second*totalForce.second);

    //! If the norm is superior to the max force admissible, rescale.
    if (abs(forceNorm)>m_maxForce)
    {
        totalForce.first = totalForce.first*m_maxForce/forceNorm;
        totalForce.first = totalForce.second*m_maxForce/forceNorm;
    }

    return totalForce;
}


//-------------------------------------------//
//-------------Setter Functions--------------//
//-------------------------------------------//

//! this method updates the parameters of the potential field
void PotentialField::setParameters(int newNuRobots, int newRho0Robots,
                                   int newNuArena, int newRho0Arena,
                                   int newZeta, int newdGoalStar,
                                   int newMaxForce, int newMaxAngle)
{
    //! store all the new parameters
    m_nuArena = newNuArena;
    m_rho0Arena = newRho0Arena;
    m_nuRobots = newNuRobots;
    m_rho0Robots = newRho0Robots;
    m_zeta = newZeta;
    m_dGoalStar = newdGoalStar;
    m_maxForce = newMaxForce;
    m_maxAngle = newMaxAngle;
}


//----------------------------------------------------------------------------//
//------------------------------Non Exported Members--------------------------//
//----------------------------------------------------------------------------//


//-------------------------------------------//
//----Setting up the Configuration Space-----//
//-------------------------------------------//


//! For Global Approach : discretize the newConfiguration Space
void PotentialField::setNewConfigurationSpace(std::vector<std::vector<State>> newConfigurationSpace)
{
    std::vector<State> row;
    State cellState;
    int col, lin, configSpaceHeight;

    //! initialize configuration space width and height
    int configSpaceWidth = newConfigurationSpace.size();

    if (!newConfigurationSpace.at(0).empty())
    {
        configSpaceHeight = newConfigurationSpace.at(0).size();
    }
    else configSpaceHeight = 0;


    //! intialize the distance between the different macro cells
    m_distCell = std::max(configSpaceHeight, configSpaceWidth)/numCells;

    if (m_distCell!= 0 && m_distCell % 2 == 0)
    {
        m_distCell--;
    }

    int step = m_distCell/2;

    //Reset configuration space width and height parameters
    m_width = 0;
    m_height = 0;

    //Iterate through the center of the new cells separated by distNodes and after
    //determinating the cell state add them to the new discretized configuration space

    for (col = step ; col<configSpaceWidth; col+=m_distCell)
    {
        row.clear();
        m_height = 0;

        for (lin = step ; lin<configSpaceHeight ; lin+=m_distCell)
        {
            //! for each macro cell, get the state and add it to the new configuration space
            cellState = getCellState(newConfigurationSpace, col, lin, step);
            row.push_back(cellState);
            m_height++;
        }
        m_configurationSpace.push_back(row);
        m_width++;
    }
}

//! For Global Approach : remove if local approach works
//! Determine whether the cell is free or occupied
State PotentialField::getCellState(std::vector<std::vector<State>> newConfigurationSpace,
                                   int column,int row,int step)
{
    //test the surronding cell to determine the state : FREE, HALLWAY or OCCUPIED
    int k, l;
    int configSpaceWidth  = newConfigurationSpace.size();
    int configSpaceHeight;

    //! get the configuration space width and height for boundary conditions
    if (!newConfigurationSpace.at(0).empty())
    {
        configSpaceHeight = newConfigurationSpace.at(0).size();
    }
    else configSpaceHeight = 0;

    State cellState = State::FREE;

    //! iterate throught the surrounding cells within the macro cell
    for (k = column-step ; k<=column+step; k++)
    {
        for (l = row-step; l<=row+step ; l++)
        {
            //! if we are in bounds identify the cell state
            if (k>=0 && l>=0 && k<configSpaceWidth && l<configSpaceHeight)
            {
                //!Case : OCCUPIED if one cell is OCCUPIED (OCCUPIED > HALLWAY > FREE)

                switch (newConfigurationSpace.at(k).at(l))
                {
                case State::FREE : //! because initialized FREE
                    break;

                case State::HALLWAY :
                    if (cellState == State::FREE)
                    {
                        cellState = State::HALLWAY;
                    }
                    break;

                case State::OCCUPIED :
                    cellState = State::OCCUPIED;
                    break;

                default :
                    break;
                }
            }
        }
    }
    return cellState;
}

//! For Global Approach : remove if local approach works
//! Identify the obstacles' borders in the configuration space
void PotentialField::identifyConfigurationSpaceBorders(std::vector<QPoint>* obstaclesBorders)
{
    int i, j, k, l;
    bool border = false;

    //! iterate though the configuration space
    for (i = 0 ; i<m_width ; i++)
    {
        for (j = 0; j<m_height ;j++)
        {
            //! if the considered cell is not occupied
            if (m_configurationSpace.at(i).at(j) == State::OCCUPIED)
            {
                //! Check the surrounding cells
                for (k = i-1 ; k<=i+1 ; k++)
                {
                    for (l = j-1 ; l<= j+1 ; l++)
                    {
                        //! if the cells are in bounds
                        if (k>=0 && l>=0 && k<m_width && l<m_height)
                        {
                            //! if one of these cells is FREE or HALLWAY
                            if (m_configurationSpace.at(k).at(l) != State::OCCUPIED)
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

//! For Global Approach : If the configuration space is changed, reinitialize all forces,
//! global and local to 0
void PotentialField::reinitializeConfigSpaceRepulsiveForces()
{
    int i, j;

    m_globalRepulsive.clear();

    //! Reset all the elements to 0
    for (i = 0 ; i<m_width ; i++)
    {
        //! Create an empty row
        std::vector<std::pair<float,float>> row;
        for (j = 0; j<m_height ;j++)
        {
            row.push_back(std::make_pair(0,0));
        }
        m_globalRepulsive.push_back(row);
    }
}


//-------------------------------------------//
//------Computing Repulsive Forces-----------//
//-------------------------------------------//

//! Method to compute all repulsive forces
std::pair<float,float> PotentialField::computeAllRepulsiveForces(int fishRobotId)
{
    std::pair<float,float> robotRepulsiveForce;
    std::pair<float,float> arenaRepulsiveForce;
    std::pair<float,float> totalRepulsiveForce;

    //! compute the total repulsive force as the sum of the force due to the robots
    robotRepulsiveForce  = computeRepulsiveForceDueToRobots(fishRobotId);

    //! and the force due to the obstacles in the configuration space
    if(m_approach == Approach::LOCAL)
    {
        //! if local compute the force
        arenaRepulsiveForce  = computeLocalRepulsiveForceDueToArena(fishRobotId);
    }
    else
    {
        //! if global access the stored value of the force at current position
        QPoint currentPos = m_fishRobots->at(fishRobotId)->getPosition();
        arenaRepulsiveForce.first  = m_globalRepulsive.at(currentPos.x()).at(currentPos.y()).first;
        arenaRepulsiveForce.second = m_globalRepulsive.at(currentPos.x()).at(currentPos.y()).second;
    }

    //! sum the two repulsive forces
    totalRepulsiveForce.first = arenaRepulsiveForce.first + robotRepulsiveForce.first;
    totalRepulsiveForce.second = arenaRepulsiveForce.second + robotRepulsiveForce.second;

    //! return the total
    return  totalRepulsiveForce;
}


//! For Local Approach : compute the repulsive force due to the arena
std::pair<float,float> PotentialField::computeLocalRepulsiveForceDueToArena(int fishRobotId)
{
    std::pair<float,float> arenaRepulsiveForce(0,0), force(0,0);
    QPoint currentPos = m_fishRobots->at(fishRobotId)->getPosition()/m_distCell;
    QPoint obstaclePos(0,0);
    int i,j;

    QPoint  deltaCoord, vObst, goalCoord;
    QPointF vRobot;

    //! Iterate through the cells locally around current position
    for (i = currentPos.x() - distInfluence/(2*m_distCell) ; i<= currentPos.x() + distInfluence/(2*m_distCell) ; i++)
    {
        for (j = currentPos.y() - distInfluence/(2*m_distCell) ; j<= currentPos.y() + distInfluence/(2*m_distCell) ; j++)
        {
            //! if the considered position is in bounds
            if (i>0 && j>0 && i<m_width && j<m_height)
            {
                //! and if the current cell is occupied
                if(m_configurationSpace.at(i).at(j) == State::OCCUPIED)
                {
                    //! identify the cell as an obstacle
                    obstaclePos.setX(i);
                    obstaclePos.setY(j);

                    //! intiailize repulsive force parameters
                    m_nu = m_nuArena;
                    m_rho0 = m_rho0Arena/m_distCell;
                    //! compute the repulsive force
                    force = computeLocalRepulsiveForce(currentPos, obstaclePos);

                    //! and sum it to the local repulsive force
                    arenaRepulsiveForce.first  += force.first;
                    arenaRepulsiveForce.second += force.second;

                }
            }
        }
    }
    return arenaRepulsiveForce;
}


//! For Global Approach : compute the global repulsive force
void PotentialField::computeGlobalRepulsiveForceDueToArena()
{
    int i, j, k;
    std::vector<QPoint> obstaclesBorders;
    std::pair<float,float> force;
    QPoint currentPos(0,0);

    //! Reset global repulsive force
    reinitializeConfigSpaceRepulsiveForces();

    //! Identify the obstacles' borders in the configuration space
    identifyConfigurationSpaceBorders(&obstaclesBorders);

    //! compute the global repulsive function

    //! Go through the discretized image
    for (i = 0; i<m_width ; i++)
    {
        for(j = 0 ; j<m_height ; j++)
        {
            //! reinitialize local force
            force.first  = 0;
            force.second = 0;

            //! update current position
            currentPos.setX(i);
            currentPos.setY(j);

            if(m_configurationSpace.at(i).at(j) != State::OCCUPIED)
            {
                //! At each free position compute the repulsive force for all border obstacles
                for (k = 0 ; k<(int)obstaclesBorders.size(); k++)
                {
                    QPoint obstaclePos = obstaclesBorders.at(k);

                    //! set the repulsive force parameters
                    m_nu = m_nuArena;
                    m_rho0 = m_rho0Arena/m_distCell;
                    force = computeLocalRepulsiveForce(currentPos, obstaclePos);

                    //! Sum the results
                    m_globalRepulsive.at(i).at(j).first  += force.first;
                    m_globalRepulsive.at(i).at(j).second += force.second;
                }
            }
            else
            {
                m_globalRepulsive.at(i).at(j).first  = 0;
                m_globalRepulsive.at(i).at(j).second = 0;
            }

        }
    }
}

//! compute the repulsive force around a robot due to the other robots
std::pair<float,float> PotentialField::computeRepulsiveForceDueToRobots(int fishRobotId)
{
    std::pair<float,float> robotsRepulsiveForce(0,0), force(0,0);
    QPoint currentPos = m_fishRobots->at(fishRobotId)->getPosition();
    QPoint obstaclePos(0,0);
    float  alphaObst, alphaRobot = m_fishRobots->at(fishRobotId)->getOrientation();
    int i;

    QPoint  deltaCoord, vObst, goalCoord;
    QPointF vRobot;

    //! for all the robots in the experiment
    for (i = 0 ; i<(int)m_fishRobots->size() ; i++)
    {
        //! different than the current fish robot
        if (i != fishRobotId)
        {
            obstaclePos = m_fishRobots->at(i)->getPosition();
            //qDebug()<<"obstacle Pos : "<<obstaclePos.x()<<" , "<<obstaclePos.y();
            //! Compute angle between obstacle and current robot
            vObst.setX(obstaclePos.x()-currentPos.x());
            vObst.setY(obstaclePos.y()-currentPos.y());
            vRobot.setX(sin(alphaRobot*DEG2RAD));  //convert from deg to radians
            vRobot.setY(-cos(alphaRobot*DEG2RAD)); //convert from deg to radians
            alphaObst = (atan2(vObst.y(),vObst.x()) - atan2(vRobot.y(),vRobot.x()))*RAD2DEG;

            //! normalize angle
            if (fabs(alphaObst) > 180)
            {
                alphaObst-= sgn(alphaObst)*360;
            }

            //! if the obstacle is in the robot's path
            if (fabs(alphaObst) < m_maxAngle)
            {
                //! compute the resulting force
                m_nu = m_nuRobots;
                m_rho0 = m_rho0Robots;

                force = computeLocalRepulsiveForce(currentPos, obstaclePos);
                //! and sum it to the local repulsive force
                robotsRepulsiveForce.first  += force.first;
                robotsRepulsiveForce.second += force.second;
            }
        }
    }
    return robotsRepulsiveForce;
}

//! compute the local repulsive force at a point due to an obstacle,
//! NOTE : the desired potential field repulsive parameters must be set prior to the call of the method
std::pair<float,float> PotentialField::computeLocalRepulsiveForce(QPoint currentPos, QPoint obstaclePos)
{
    std::pair<float,float> localRepulsiveForce(0,0);
    QPoint deltaPos;
    float   rhoObst;

    //! compute the distance between the current position and the obstacle
    deltaPos = currentPos - obstaclePos;
    rhoObst = sqrt(pow(deltaPos.x(), 2) + pow(deltaPos.y(), 2));

    //! if the distance is inferior to the limit
    if (rhoObst < m_rho0)
    {
        //! compute the repulsive force
        localRepulsiveForce.first  = m_nu*(1/rhoObst - 1/m_rho0)*deltaPos.x()/pow(rhoObst,3) ;
        localRepulsiveForce.second = m_nu*(1/rhoObst - 1/m_rho0)*deltaPos.y()/pow(rhoObst,3) ;
    }
    else
    {
        //! else set the repulsive force to 0
        localRepulsiveForce.first  = 0 ;
        localRepulsiveForce.second = 0 ;
    }

    return localRepulsiveForce;
}

//-------------------------------------------//
//------Computing Attractive Forces----------//
//-------------------------------------------//

//! compute the attractive force by the target on it's robot
//! NOTE : the desired potnetial field attactive parameters must be set prior to the call of the method
std::pair<float,float> PotentialField::computeAttractiveForce(int fishRobotId, QPoint targetPos)
{
    std::pair<float,float> attractiveForce(0,0);
    QPoint currentPos, deltaPos;
    float dGoal ;

    //! Compute the distance between current robot and it's target
    currentPos = m_fishRobots->at(fishRobotId)->getPosition();
    deltaPos = currentPos - targetPos;
    dGoal = sqrt(pow(deltaPos.x(), 2) + pow(deltaPos.y(), 2));

    //! if the distance is inferior to the limit
    if (dGoal <= m_dGoalStar)
    {
        //! compute the strong attractive force
        attractiveForce.first  = -m_zeta*deltaPos.x();
        attractiveForce.second = -m_zeta*deltaPos.y();
    }
    else
    {
        //! else compute the "weaker" attractive force
        attractiveForce.first  = - m_dGoalStar*m_zeta*deltaPos.x()/dGoal;
        attractiveForce.second = - m_dGoalStar*m_zeta*deltaPos.y()/dGoal;
    }

    return attractiveForce;
}

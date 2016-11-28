#include "potentialfield.h"

const int numCells      = 100;

const int distInfluence = 150;

const int robotsInfluence = 40;


//----------------------------------------------------------------------------//
//-------------------------------Class Constructors---------------------------//
//----------------------------------------------------------------------------//

PotentialField::PotentialField(std::vector<std::vector<enum State>> configurationSpace,
                               std::vector<FishRobot*>* fishRobots):m_zeta(2), m_dGoalStar(50),
                                                                    m_nuArena(300), m_rho0Arena(100),
                                                                    m_nuRobots(1000),m_rho0Robots(30),
                                                                    m_maxForce(1000),m_maxAngle(60)
{
    //! Store fishRobots and Target pointers
    m_fishRobots = fishRobots;


    //-------
    m_configurationSpace = configurationSpace;



    m_width = configurationSpace.size();

    if(configurationSpace.empty())
    {
        m_height = 0;
    }
    else
    {
        m_height = configurationSpace.at(0).size();
    }
    //-------

    /*
    qDebug()<<"m_fishRobots size"<<m_fishRobots->size();

    //! set up the new discretized configuration space (to reduce computation time)
    setNewConfigurationSpace(configurationSpace);
    qDebug()<<"width : "<<m_width<<" height : "<<m_height;

    //! Compute new global repulsive force
    computeConfigSpaceRepulsiveForce();
    */
}

//----------------------------------------------------------------------------//
//--------------------------------Exported Members----------------------------//
//----------------------------------------------------------------------------//

//! Compute the total force on a robot, both attractive and repulsive
std::pair<float,float> PotentialField::computeTotalForceForRobot(int fishRobotId)
{
    std::pair<float,float> totalForce;
    std::pair<float,float> robotRepulsiveForce;
    std::pair<float,float> arenaRepulsiveForce;
    std::pair<float,float> attractiveForce;
    float forceNorm;

    QPoint currentPos = m_fishRobots->at(fishRobotId)->getPosition();///m_distCell;

    //compute the total repulsive force as the sum of the force due to the robots
    robotRepulsiveForce  = computeRepulsiveForceDueToRobots(fishRobotId);
    // and the force due to the obstacles in the configuration space

    arenaRepulsiveForce  = computeRepulsiveForceDueToArena(fishRobotId);
    /*
    //! TODO : prendre la moyenne des points autour?
    totalRepulsiveForce.first  += m_globalRepulsive.at(currentPos.x()).at(currentPos.y()).first;
    totalRepulsiveForce.second += m_globalRepulsive.at(currentPos.x()).at(currentPos.y()).second;
    */

    //the total attractive force at the current position
    attractiveForce = computeAttractiveForce(fishRobotId);

    //the total force is given by the total repulsive and attractive forces
    totalForce.first = arenaRepulsiveForce.first + attractiveForce.first + robotRepulsiveForce.first;
    totalForce.second = arenaRepulsiveForce.second + attractiveForce.second + robotRepulsiveForce.second;

    qDebug()<<"TARGET  force : "<<attractiveForce.first<<" ; "<< attractiveForce.second;
    qDebug()<<"ARENA   force : "<<arenaRepulsiveForce.first<<" ; "<< arenaRepulsiveForce.second;
    qDebug()<<"ROBOTS  force : "<<robotRepulsiveForce.first<<" ; "<< robotRepulsiveForce.second;
    qDebug()<<"TOTAL   force : "<<totalForce.first<<" ; "<< totalForce.second;

    forceNorm = sqrt(totalForce.first*totalForce.first+totalForce.second*totalForce.second);

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

//! FIXME : remove if local approach works
//! Discretize the newConfiguration Space
void PotentialField::setNewConfigurationSpace(std::vector<std::vector<State>> newConfigurationSpace)
{
    std::vector<State> row;
    State cellState;
    int col, lin, configSpaceHeight;

    int configSpaceWidth = newConfigurationSpace.size();

    if (!newConfigurationSpace.at(0).empty())
    {
        configSpaceHeight = newConfigurationSpace.at(0).size(); // FIXME : dangerous, can crash here
    }
    else configSpaceHeight = 0;


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
    //determinating the cell state add them to the new configuration space

    for (col = step ; col<configSpaceWidth; col+=m_distCell)
    {
        row.clear();
        m_height = 0;

        for (lin = step ; lin<configSpaceHeight ; lin+=m_distCell)
        {
            cellState = getCellState(newConfigurationSpace, col, lin, step);
            row.push_back(cellState);
            m_height++;
        }
        m_configurationSpace.push_back(row);
        m_width++;
    }
}

//! FIXME : remove if local approach works
//! Determine whether the cell is free or occupied
State PotentialField::getCellState(std::vector<std::vector<State>> newConfigurationSpace,
                                int column,int row,int step)
{
    //test the surronding cell to determine the state : FREE, HALLWAY or OCCUPIED
    int k, l;
    int configSpaceWidth  = newConfigurationSpace.size();
    int configSpaceHeight;

    if (!newConfigurationSpace.at(0).empty())
    {
        configSpaceHeight = newConfigurationSpace.at(0).size(); // FIXME : dangerous, can crash here
    }
    else configSpaceHeight = 0;

    State cellState = State::FREE;

    for (k = column-step ; k<=column+step; k++)
    {
        for (l = row-step; l<=row+step ; l++)
        {
            //if we are in bounds
            if (k>=0 && l>=0 && k<configSpaceWidth && l<configSpaceHeight)
            {
                //Case : OCCUPIED if one cell is OCCUPIED (OCCUPIED > HALLWAY > FREE)

                switch (newConfigurationSpace.at(k).at(l))
                {
                case State::FREE : //because initialized FREE
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

//! FIXME : remove if local approach works
//! Identify the obstacles' borders in the configuration space
void PotentialField::identifyConfigurationSpaceBorders(std::vector<QPoint>* obstaclesBorders)
{
    int i, j, k, l;
    bool border = false;

    for (i = 0 ; i<m_width ; i++)
    {
        for (j = 0; j<m_height ;j++)
        {
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
                            //if one of these cells is FREE or HALLWAY
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

//! FIXME : remove if local approach works
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

//-------------------------------------------//
//------Computing Repulsive Forces-----------//
//-------------------------------------------//

//! compute the repulsive force due to the arena
//! FIXME : put in compute local repulsive function if local approach used
std::pair<float,float> PotentialField::computeRepulsiveForceDueToArena(int fishRobotId)
{
    std::pair<float,float> arenaRepulsiveForce(0,0), force(0,0);
    QPoint currentPos = m_fishRobots->at(fishRobotId)->getPosition();
    QPoint obstaclePos(0,0);
    float  alphaObst, alphaRobot = m_fishRobots->at(fishRobotId)->getOrientation();
    int i,j;

    QPoint  deltaCoord, vObst, goalCoord;
    QPointF vRobot;

    //qDebug()<<"fish robots : "<<m_fishRobots->size()<<" ID : "<<fishRobotId;

    for (i = currentPos.x() - distInfluence/2 ; i<= currentPos.x() + distInfluence/2 ; i++)
    {
        for (j = currentPos.y() - distInfluence/2 ; j<= currentPos.y() + distInfluence/2 ; j++)
        {
            if (i>0 && j>0 && i<m_width && j<m_height)
            {
                if(m_configurationSpace.at(i).at(j)== State::OCCUPIED)
                {
                    obstaclePos.setX(i);
                    obstaclePos.setY(j);

                    m_nu = m_nuArena;
                    m_rho0 = m_rho0Arena;
                    force = computeLocalRepulsiveForce(currentPos, obstaclePos);

                    //! and sum it to the local repulsive force
                    arenaRepulsiveForce.first  += force.first;
                    arenaRepulsiveForce.second += force.second;

                }
            }
            //qDebug()<<"obstacle Pos : "<<obstaclePos.x()<<" , "<<obstaclePos.y();
        }
    }
    return arenaRepulsiveForce;
}


//! FIXME : remove if local approach works
//! compute the global repulsive force for all robots
void PotentialField::computeConfigSpaceRepulsiveForce()
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

                    m_nu = m_nuArena;
                    m_rho0 = m_rho0Arena;
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

            //qDebug()<<"i : "<<i<<" j : "<<j;
            //qDebug()<<"force : "<<m_globalRepulsive.at(i).at(j).first<<" , "<<m_globalRepulsive.at(i).at(j).second;
        }
    }
}

//! compute the repulsive force around a robot due to the other robots
std::pair<float,float> PotentialField::computeRepulsiveForceDueToRobots(int fishRobotId)
{
    std::pair<float,float> robotsRepulsiveForce(0,0), force(0,0);
    QPoint currentPos = m_fishRobots->at(fishRobotId)->getPosition();///m_distCell;
    QPoint obstaclePos(0,0);
    float  alphaObst, alphaRobot = m_fishRobots->at(fishRobotId)->getOrientation();
    int i;

    QPoint  deltaCoord, vObst, goalCoord;
    QPointF vRobot;

    //qDebug()<<"fish robots : "<<m_fishRobots->size()<<" ID : "<<fishRobotId;

    for (i = 0 ; i<(int)m_fishRobots->size() ; i++)
    {
        if (i != fishRobotId)
        {
            obstaclePos = m_fishRobots->at(i)->getPosition();///m_distCell;
            qDebug()<<"obstacle Pos : "<<obstaclePos.x()<<" , "<<obstaclePos.y();
            //Compute angle between obstacle and current robot
            vObst.setX(obstaclePos.x()-currentPos.x());
            vObst.setY(obstaclePos.y()-currentPos.y());
            vRobot.setX(sin(alphaRobot*DEG2RAD));  //pour avoir des radians
            vRobot.setY(-cos(alphaRobot*DEG2RAD)); //pour avoir des radians
            alphaObst = (atan2(vObst.y(),vObst.x()) - atan2(vRobot.y(),vRobot.x()))*RAD2DEG;

            //normalize angle
            if (fabs(alphaObst) > 180)
            {
                alphaObst-= sgn(alphaObst)*360;
            }

            //if the obstacle is in the robot's path
            if (fabs(alphaObst) < m_maxAngle)
            {
                //! compute the resulting force
                m_nu = m_nuRobots;
                m_rho0 = m_rho0Robots;


                for (int j = currentPos.x()-robotsInfluence ; j<currentPos.x()+robotsInfluence ; j++)
                {
                    for (int k = currentPos.y()-robotsInfluence; k<currentPos.y()+robotsInfluence; k++)
                    {
                        if (k>=0 && j>=0 && j<m_width && k<m_height)
                        {
                            force = computeLocalRepulsiveForce(currentPos, obstaclePos);
                            //! and sum it to the local repulsive force
                            robotsRepulsiveForce.first  += force.first;
                            robotsRepulsiveForce.second += force.second;
                        }
                    }
                }
            }
        }
    }
    return robotsRepulsiveForce;
}

//! compute the local repulsive force at a point due to an obstacle
std::pair<float,float> PotentialField::computeLocalRepulsiveForce(QPoint currentPos, QPoint obstaclePos)
{
    std::pair<float,float> localRepulsiveForce(0,0);
    QPoint deltaPos;
    float   rhoObst;

    deltaPos = currentPos - obstaclePos;

    rhoObst = sqrt(pow(deltaPos.x(), 2) + pow(deltaPos.y(), 2));

    if (rhoObst < m_rho0)
    {
        localRepulsiveForce.first  = m_nu*(1/rhoObst - 1/m_rho0)*deltaPos.x()/pow(rhoObst,3) ;
        localRepulsiveForce.second = m_nu*(1/rhoObst - 1/m_rho0)*deltaPos.y()/pow(rhoObst,3) ;
    }
    else
    {
        localRepulsiveForce.first  = 0 ;
        localRepulsiveForce.second = 0 ;
    }

    return localRepulsiveForce;
}

//-------------------------------------------//
//------Computing Attractive Forces----------//
//-------------------------------------------//

//! compute the attractive force by the target on it's robot
std::pair<float,float> PotentialField::computeAttractiveForce(int fishRobotId)
{
    std::pair<float,float> attractiveForce(0,0);
    QPoint currentPos, targetPos, deltaPos;
    float dGoal ;

    currentPos = m_fishRobots->at(fishRobotId)->getPosition();//m_distCell;
    targetPos  = m_fishRobots->at(fishRobotId)->getTargetPosition();///m_distCell;
    deltaPos = currentPos - targetPos;

    dGoal = sqrt(pow(deltaPos.x(), 2) + pow(deltaPos.y(), 2));


    if (dGoal <= m_dGoalStar)
    {
        attractiveForce.first  = -m_zeta*deltaPos.x();
        attractiveForce.second = -m_zeta*deltaPos.y();
    }
    else
    {
        attractiveForce.first  = - m_dGoalStar*m_zeta*deltaPos.x()/dGoal;
        attractiveForce.second = - m_dGoalStar*m_zeta*deltaPos.y()/dGoal;
    }

    return attractiveForce;
}


//temps passer corridor, chemin plus court, temps.... -> LUNDI

//Autor : Laila El Hamamsy
//Date Created : Sunday December 4th 2016
//Version : 1
//Last Modified :


#include "dynamicwindow.h"


//----------------------------------------------------------------------------//
//-------------------------------Class Constructors---------------------------//
//----------------------------------------------------------------------------//

DynamicWindow::DynamicWindow(std::vector<std::vector<enum State>> configurationSpace,
                             std::vector<FishRobot*> *fishRobots): m_alpha(30),
                                                                   m_timeframe(dtSamples*simulation_dt),
                                                                   m_beta(1),
                                                                   m_gamma(1),
                                                                   m_delta(1)
{
    //! Store fishRobots and Target pointers
    m_fishRobots = fishRobots;

    //! Store the configuration space
    m_configurationSpace = configurationSpace;

    //! Store the Dimensions of the configuration space
    m_width = configurationSpace.size();

    if(configurationSpace.empty())
    {
        m_height = 0;
    }
    else
    {
        m_height = configurationSpace.at(0).size();
    }
}


//----------------------------------------------------------------------------//
//--------------------------------Exported Members----------------------------//
//----------------------------------------------------------------------------//

//! Compute the new linear and angular velocities for the given robot if an obstacle is in the vicinity
//! if no obstacle is detected it returns -1,-1
std::pair<float,float> DynamicWindow::computeNewLinearAndAngularVelIfObstacle(int fishRobotId, QPoint pathGoal)
{
    std::pair<float,float> optimalVel(-1,-1); //FIXME : -1 in float... -.-

    if (fishRobotId == 1)
    {
        return std::make_pair(0,0);
    }

    m_fishRobotId   = fishRobotId;
    m_goal          = pathGoal;
    m_pos           = m_fishRobots->at(m_fishRobotId)->getPosition();
    m_vel           = m_fishRobots->at(m_fishRobotId)->getLinearVelocity();
    m_angularVel    = m_fishRobots->at(m_fishRobotId)->getAngularVelocity()*DEG2RAD;
    m_maxVel        = m_fishRobots->at(m_fishRobotId)->getMaxLinearVelocity();
    m_angle         = m_fishRobots->at(m_fishRobotId)->getOrientation()*DEG2RAD;
    m_maxAngularVel = m_fishRobots->at(m_fishRobotId)->getMaxAngularVelocity()*DEG2RAD;

    // qDebug()<<" m_vel"<<m_vel;
    // qDebug()<<" m_angularVel"<<m_angularVel;

    m_admissibleVelocities.clear();

    if(!robotCloseby(m_pos))
    {
        //qDebug()<<"no robot closeby";
        return optimalVel;
    }

    initializeRobotSpace();

    qDebug()<<"robot closeby";
    searchSpaceForAdmissibleVelocities();

    optimalVel = identifyOptimalVelocities();
    qDebug()<<"----------------------------------------------";
    qDebug()<<"----------------------------------------------";
    qDebug()<<"----------------------------------------------";
    qDebug()<<"----------------------------------------------";
    qDebug()<<optimalVel.first<<optimalVel.second;
    qDebug()<<"----------------------------------------------";
    qDebug()<<"----------------------------------------------";
    qDebug()<<"----------------------------------------------";
    qDebug()<<"----------------------------------------------";
    return optimalVel;
}

//-------------------------------------------//
//-------------Setter Functions--------------//
//-------------------------------------------//

//! this method updates the parameters of the dynamic window
void DynamicWindow::setParameters(int newAlpha, int newBeta, int newGamma, int newDelta, float newTimeframe)
{
    m_alpha = newAlpha;
    m_beta  = newBeta;
    m_gamma = newGamma;
    m_delta = newDelta;
    m_timeframe  = newTimeframe;
}


//----------------------------------------------------------------------------//
//------------------------------Non Exported Members--------------------------//
//----------------------------------------------------------------------------//

//! this method positions the fish robot in their own configuration space to
//! take into account the robot dimensions for the obstacle avoidance.
void DynamicWindow::initializeRobotSpace()
{
    std::vector<State> row;
    QPoint robotPos;
    int maxRobotDimension = 3*std::max(m_fishRobotHeight, m_fishRobotWidth);


    for (int i = 0 ; i<m_width; i++)
    {
        for (int j = 0; j<m_height ; j++)
        {
            row.push_back(State::FREE);
        }
        m_robotsSpace.push_back(row);
        row.clear();
    }

    //! go through the list of all the robots in the simluation
    for (int i = 0 ; i<(int)m_fishRobots->size() ; i++)
    {
        //! if the considered robot is not the current robot
        if (i != m_fishRobotId)
        {
            robotPos = m_fishRobots->at(i)->getPosition();
            for(int k =  robotPos.x()-maxRobotDimension ; k<=robotPos.x()+maxRobotDimension; k++)
            {
                for(int l =  robotPos.y()-maxRobotDimension ; l<=robotPos.y()+maxRobotDimension; l++)
                {
                    //! if the new position is in bounds
                    if(k>=0 && l>=0 && k<m_width && l<m_height)
                    {
                        m_robotsSpace.at(k).at(l) = State::OCCUPIED;
                    }
                }
            }
            qDebug()<<"robotPos"<< robotPos.x()<< robotPos.y();
            qDebug()<<"k"<< robotPos.x()-maxRobotDimension<< robotPos.x()+maxRobotDimension;
            qDebug()<<"l"<< robotPos.y()-maxRobotDimension<< robotPos.y()+maxRobotDimension;
        }
    }
}

//! this method identifies whether or not there is an obstacle on the current
//! path
bool DynamicWindow::robotCloseby(QPoint currentPos, int distRatio)
{
    int     fishRobotWidth  = m_fishRobots->at(m_fishRobotId)->getFishRobotWidth();
    int     fishRobotHeight = m_fishRobots->at(m_fishRobotId)->getFishRobotHeight();
    QPoint  deltaPos, obstacleRobot;
    float   dist;
    int     distLimit = 2*std::max(fishRobotWidth, fishRobotHeight)/distRatio;

    //! go through the list of all the robots in the simluation
    for (int i = 0 ; i<(int)m_fishRobots->size() ; i++)
    {
        //! if the considered robot is not the current robot
        if (i != m_fishRobotId)
        {
            //! calculate the distance between the robots
            obstacleRobot = m_fishRobots->at(i)->getPosition();
            deltaPos.setX(currentPos.x()-obstacleRobot.x());
            deltaPos.setY(currentPos.y()-obstacleRobot.y());
            dist = sqrt(deltaPos.x()*deltaPos.x()+deltaPos.y()*deltaPos.y());

            //! if it is too close
            if (dist<distLimit)
            {
                QPointF vObst, vRobot;
                float angleToGoal;

                //! Compute angle between obstacle and current robot
                vObst.setX(m_goal.x()-currentPos.x());
                vObst.setY(m_goal.y()-currentPos.y());
                vRobot.setX(sin(m_angle));  //! angle in radians
                vRobot.setY(-cos(m_angle)); //! angle in radians
                angleToGoal = (atan2(vObst.y(),vObst.x()) - atan2(vRobot.y(),vRobot.x())); //! in radians

                //! normalize the angle
                while (fabs(angleToGoal) > M_PI)
                {
                    angleToGoal-= sgn(angleToGoal)*2*M_PI;
                }

                if(fabs(angleToGoal)<M_PI/4)
                {
                    return true;
                }
            }
        }
    }

    //! if no other robot has been detected as too close
    return false;
}

//! this method goes through the search space to identify all admissible
//! velocities, ie the ones that do not lead to collision within a certain timeframe

void DynamicWindow::searchSpaceForAdmissibleVelocities()
{
    // qDebug()<<"ADMISSIBLE VELOCITIES";
    //! compute the dynamic frame
    float minLinearVel  = m_vel-linearAcceleration*m_timeframe;
    float maxLinearVel  = m_vel+linearAcceleration*m_timeframe;
    float minAngularVel = m_angularVel  - angularAcceleration* m_timeframe;
    float maxAngularVel = m_angularVel  + angularAcceleration* m_timeframe;

    qDebug()<<" minLinearVel"<<minLinearVel<<" maxLinearVel"<<maxLinearVel;
    qDebug()<<" minAngVel"<<minAngularVel<<" maxAngVel"<<maxAngularVel;

    minLinearVel  = std::max(minLinearVel,(float)0);
    maxLinearVel  = std::min(maxLinearVel, (float)m_maxLinearVel);

    minAngularVel  = std::max(minAngularVel,-(float)m_maxAngularVel);
    maxAngularVel  = std::min(maxAngularVel, (float)m_maxAngularVel);

    qDebug()<<" minLinearVel"<<minLinearVel<<" maxLinearVel"<<maxLinearVel;
    qDebug()<<" minAngVel"<<minAngularVel<<" maxAngVel"<<maxAngularVel;

    //! compute the dynamic window
    for (int i = 0; i<=numberLinearVel ; i++)
    {
        float linearVel = minLinearVel + (maxLinearVel - minLinearVel)*i/numberLinearVel;

        if(linearVel<m_maxLinearVel)
        {
            for(int j = 0; j<=numberAngularVel; j++)
            {
                float angularVel = minAngularVel + (maxAngularVel-minAngularVel)*j/numberAngularVel;
                //qDebug()<<"TESTING v "<<linearVel<<" omega"<<angularVel;
                if (fabs(angularVel) <= m_maxAngularVel)
                {
                    //qDebug()<<" admissible velocities : testing distance function :";
                    float distance = distanceTravelled(linearVel,angularVel);


                    //! if the velocity lets the robot stop in time to avoid collision
                    if (  fabs(linearVel)  <= sqrt(2*distance*linearAcceleration)
                          && fabs(angularVel) <= sqrt(2*distance*angularAcceleration)
                          && (fabs(linearVel)> 0.0001 || fabs(angularVel)>0.0001))
                    {
                        //qDebug()<<" SUCCESS v"<<linearVel<<"omega"<<angularVel;
                        m_admissibleVelocities.push_back(std::make_tuple(linearVel,angularVel,distance));
                    }
                    else
                    {
                        //qDebug()<<" can't stop in time : distance to closest obstacle : "<<distance;
                        //qDebug()<<" linear vel"<<linearVel<<"not <"<<sqrt(2*distance*linearAcceleration)<<" or angular vel"<<angularVel<<"not <"<<sqrt(2*distance*angularAcceleration);
                    }
                }
                //else qDebug()<<" angular vel too high "<<angularVel;
            }
        }
    }

    if(m_admissibleVelocities.empty())
    {
        qDebug()<<"--------------------PROBLEM--------------------";
    }
}

//! this method computes the distance travelled within a certain timeframe
//! or the distance to the closest object given a certain velocity
float DynamicWindow::distanceTravelled(int v, int omega)
{
    float totalDistance = 0, vx, vy;
    float newAngle = m_alpha; //! in radians
    int maxRobotDimension = 0; std::max(m_fishRobotHeight, m_fishRobotWidth);
    QPointF dPos(0,0), newPos(m_pos);

    for (int i = 1; i<=dtSamples ; i++)
    {
        //! compute new robot orientation
        newAngle += omega*simulation_dt; //! in radians
        //! normalize the angle
        while (fabs(newAngle) > M_PI)
        {
            newAngle-= sgn(newAngle)*2*M_PI;
        }
        //! compute new translational velocities
        vx = v*sin(newAngle) ;
        vy = -v*cos(newAngle);
        //! compute new position
        dPos.setX(simulation_dt*vx);
        dPos.setY(simulation_dt*vy);
        newPos += dPos;

        for (int k = newPos.x()-maxRobotDimension/2 ; k<=newPos.x()+maxRobotDimension/2 ; k++)
        {
            for (int l = newPos.y()-maxRobotDimension/2 ; l<=newPos.y()+maxRobotDimension/2 ; l++)
            {
                //! if the new position is in bounds
                if(k>=0 && l>=0 && k<m_width && l<m_height)
                {
                    //! variable to avoid conversion issues from QPointF-> QPoint
                    QPoint currentPos(newPos.x(), newPos.y());

                    //! check to see if the configuration space is occupied
                    if(m_configurationSpace.at(k).at(l) == State::OCCUPIED
                            || m_robotsSpace.at(k).at(l) == State::OCCUPIED)
                    {
                        //! compute the total distance travelled along the arc
                        //qDebug()<<"start : "<<m_pos.x()<<m_pos.y()<<"finish : "<<newPos.x()<<newPos.y()<<"distance :"<<distVOmega;
                        //qDebug()<<"OCCUPIED v : "<<v<<"omega : "<<omega<<"distance :"<<distVOmega;
                        return totalDistance;
                    }
                    //else{qDebug()<<"Arena blocking";}

                }
                //! if the new position is outside the configuration space
                else
                {
                    //qDebug()<<"OUT OF BOUNDS v : "<<v<<"omega : "<<omega<<"distance :"<<distVOmega;
                    return totalDistance;
                }
            }
        }
        totalDistance += simulation_dt*sqrt(vx*vx+vy*vy);
        //qDebug()<<"newpos"<<newPos.x()<<newPos.y()<<" Total Distance : "<<totalDistance;
    }

    //qDebug()<<"return"<<distVOmega;
    //qDebug()<<"NO COLLISION v : "<<v<<"omega : "<<omega<<"distance :"<<distVOmega;
    return totalDistance;
}

//! this method chooses the most optimal linear and angular velocities by using
//! an objective function and choosing the highest scoring combination
std::pair<float,float> DynamicWindow::identifyOptimalVelocities()
{
    std::tuple<float,float,float> optimum;
    std::pair<float,float> optimalVelocity;
    int optimalValue = 0;
    int value;

    for (int i = 0 ; i<(int)m_admissibleVelocities.size(); i++)
    {
        value = computeObjectiveFunction(m_admissibleVelocities.at(i));
        if (value>optimalValue)
        {
            optimalValue = value;
            optimum = m_admissibleVelocities.at(i);
            optimalVelocity = std::make_pair(std::get<0>(optimum),std::get<1>(optimum));
        }
    }
    return optimalVelocity;
}

//! this method computes the total objective function for a given linear and angular
//! velocity
float DynamicWindow::computeObjectiveFunction(std::tuple<float,float,float> velocity)
{
    float value, newAngle,vx,vy;
    QPointF newPos(m_pos), dPos(0,0);

    qDebug()<<"-------------------";
    qDebug()<<"v : "<<std::get<0>(velocity)<<" omega : "<<std::get<1>(velocity);
    //! compute new robot orientation
    newAngle = m_angle + std::get<1>(velocity)*simulation_dt; //! in radians
    //qDebug()<<"m_angle"<<m_angle*RAD2DEG<<"delta angle "<<velocity.second*simulation_dt*RAD2DEG<<"newAngle"<<newAngle*RAD2DEG;

    //! normalize the angle
    while (fabs(newAngle) > M_PI)
    {
        newAngle-= sgn(newAngle)*2*M_PI;
    }

    //! compute new translational velocities
    vx =  std::get<0>(velocity)*sin(newAngle);
    vy = -std::get<0>(velocity)*cos(newAngle);
    //! compute new position
    dPos.setX(simulation_dt*vx);
    dPos.setY(simulation_dt*vy);
    newPos += dPos;

    qDebug()<<"newPos"<<newPos.x()<<newPos.y()<<"newAngle"<<newAngle;

    value = m_alpha*computeAlignFunction(newPos, newAngle)
          + m_beta *computeVelocityFunction(velocity)
          + m_gamma*computeGoalFunction(newPos)
          + m_delta*computeDistanceFunction(velocity);
    qDebug()<<"value"<<value;
    return value;
}

//! this method computes the align objective function for a given linear and angular
//! velocity to insure the robot stays directed towards the goal
float DynamicWindow::computeAlignFunction(QPointF newPos, float newAngle)
{
    float value, angleToGoal;
    QPointF vObst, vRobot;

    //! Compute angle between obstacle and current robot
    vObst.setX(m_goal.x()-newPos.x());
    vObst.setY(m_goal.y()-newPos.y());
    vRobot.setX(sin(newAngle));  //! angle in radians
    vRobot.setY(-cos(newAngle)); //! angle in radians
    angleToGoal = (atan2(vObst.y(),vObst.x()) - atan2(vRobot.y(),vRobot.x())); //! in radians

    //! normalize the angle
    while (fabs(angleToGoal) > M_PI)
    {
        angleToGoal-= sgn(angleToGoal)*2*M_PI;
    }

    //! compute the value
    value = 1-fabs(angleToGoal)/M_PI;
    qDebug()<<"align : "<<value;
    return value;
}

//! this method computes the velocity objective function for a given linear and angular
//! velocity to insure the robot prefers fastforward motion
float DynamicWindow::computeVelocityFunction(std::tuple<float,float,float> velocity)
{
    float value;
    float dist, distLimit;
    QPoint deltaPos;

    //! compute distance to the goal
    deltaPos.setX(m_pos.x() - m_goal.x());
    deltaPos.setY(m_pos.y() - m_goal.y());
    dist = sqrt(deltaPos.x()*deltaPos.x() + deltaPos.y()*deltaPos.y());

    //! compute the limit
    distLimit = std::max(m_width, m_height)/2;

    //! compare to the limit to compute the correct value
    if (dist<distLimit)
    {
        value = std::get<0>(velocity)/m_maxVel;
    }
    else value = 1 - std::get<0>(velocity)/m_maxVel;

    //! return the correct value
    qDebug()<<"velocity : "<<value;
    return value;
}

//! this method computes the goal objective function for a given position
//! velocity to insure the robot keeps going towards the goal
int DynamicWindow::computeGoalFunction(QPointF newPos)
{
    float dist, distLimit;
    QPoint deltaPos;

    //! compute distance to the goal
    deltaPos.setX(newPos.x() - m_goal.x());
    deltaPos.setY(newPos.y() - m_goal.y());
    dist = sqrt(deltaPos.x()*deltaPos.x() + deltaPos.y()*deltaPos.y());

    //! compute the limit
    distLimit = std::max(m_width, m_height)/3;

    //! compare to the limit to compute the correct binary value
    if (dist<distLimit)
    {
        qDebug()<<"Goal : 1";
        return 1;
    }

    qDebug()<<"Goal : 0";
    return 0;
}

//! this method computes the distance objective function for a given localization
//! and velocity. It returns the longest distance the robot can go without
//! collision with respect to the total distance it can travel within the given timeframe
float DynamicWindow::computeDistanceFunction(std::tuple<float,float,float> velocity)
{
    float distance = 0;
    float newAngle = m_alpha; //! in radians
    int maxRobotDimension = std::max(m_fishRobotHeight, m_fishRobotWidth);
    QPointF dPos(0,0), newPos(m_pos);

    //! Convert it to a value between 0 and 1
    if (fabs(std::get<0>(velocity)) < 0.00001)
    {
        distance = 0;
    }
    else
    {
        distance = std::get<2>(velocity)/(std::get<0>(velocity)*m_timeframe);
    }

    qDebug()<<"Distance : "<<distance;

    return distance;
}

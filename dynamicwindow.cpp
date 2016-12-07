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
                                                                   m_gamma(1)
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
    m_angularVel    = m_fishRobots->at(m_fishRobotId)->getAngularVelocity();
    m_maxVel        = m_fishRobots->at(m_fishRobotId)->getMaxLinearVelocity();
    m_angle         = m_fishRobots->at(m_fishRobotId)->getOrientation();
    m_maxAngularVel = m_fishRobots->at(m_fishRobotId)->getMaxAngularVelocity();

    qDebug()<<" m_vel"<<m_vel;
    qDebug()<<" m_angularVel"<<m_angularVel;

    m_admissibleVelocities.clear();

    if(!robotCloseby(m_pos))
    {
        qDebug()<<"no robot closeby";
        return optimalVel;
    }

    qDebug()<<"robot closeby";
    searchSpaceForAdmissibleVelocities();
    optimalVel = identifyOptimalVelocities();

    return optimalVel;
}

//-------------------------------------------//
//-------------Setter Functions--------------//
//-------------------------------------------//

//! this method updates the parameters of the dynamic window
void DynamicWindow::setParameters(int newAlpha, int newBeta, int newGamma, float newTimeframe)
{
    m_alpha = newAlpha;
    m_beta  = newBeta;
    m_gamma = newGamma;
    m_timeframe  = newTimeframe;
}


//----------------------------------------------------------------------------//
//------------------------------Non Exported Members--------------------------//
//----------------------------------------------------------------------------//

//! this method identifies whether or not there is an obstacle on the current
//! path
bool DynamicWindow::robotCloseby(QPoint currentPos, int distRatio)
{
    int     fishRobotWidth = m_fishRobots->at(m_fishRobotId)->getFishRobotWidth();
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
                return true;
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
    qDebug()<<"ADMISSIBLE VELOCITIES";
    //! compute the dynamic frame
    float minLinearVel  = m_vel-linearAcceleration*m_timeframe;
    float maxLinearVel  = m_vel+linearAcceleration*m_timeframe;
    float minAngularVel = m_angularVel  - angularAcceleration* m_timeframe;
    float maxAngularVel = m_angularVel  + angularAcceleration* m_timeframe;

    qDebug()<<" BEFORE";
    qDebug()<<" minLinearVel"<<minLinearVel;
    qDebug()<<" maxLinearVel"<<maxLinearVel;

    minLinearVel  = std::max(minLinearVel,(float)0);

    qDebug()<<" AFTER";
    qDebug()<<" minLinearVel"<<minLinearVel;
    qDebug()<<" maxLinearVel"<<maxLinearVel;
    qDebug()<<" my maxLinearVel"<<m_maxLinearVel;
    qDebug()<<" float my maxLinearVel"<<(float)m_maxLinearVel;

    //! compute the dynamic window
    for (int i = 0; i<=numberLinearVel ; i++)
    {
        float linearVel = minLinearVel + (maxLinearVel - minLinearVel)*i/numberLinearVel;

        if(linearVel<m_maxLinearVel)
        {
            for(int j = 0; j<=numberAngularVel; j++)
            {
                float angularVel = minAngularVel + (maxAngularVel-minAngularVel)*i/numberAngularVel;

                if (fabs(angularVel) <= m_maxAngularVel)
                {
                    //qDebug()<<" admissible velocities : testing distance function :";
                    float distance = dist(linearVel,angularVel);


                    //! if the velocity lets the robot stop in time to avoid collision
                    if (  fabs(linearVel)  <= sqrt(2*distance*linearAcceleration)
                       && fabs(angularVel) <= sqrt(2*distance*angularAcceleration))
                    {
                        qDebug()<<" SUCCESS v"<<linearVel<<"omega"<<angularVel;
                        m_admissibleVelocities.push_back(std::make_pair(linearVel,angularVel));
                    }
                    else
                    {
                        qDebug()<<" distance to closest obstacle : "<<distance;
                        qDebug()<<" linear vel"<<linearVel<<"not <"<<sqrt(2*distance*linearAcceleration);
                        qDebug()<<" angular vel"<<angularVel<<"not <"<<sqrt(2*distance*angularAcceleration);
                        qDebug()<<" can't stop in time";
                    }
                }
                else qDebug()<<" angular vel too high "<<angularVel;
            }
        }
    }

    if(m_admissibleVelocities.empty())
    {
        qDebug()<<"--------------------PROBLEM--------------------";
    }
}

//! this method computes the
//! distance to the closest object on the corresponding
//! curvature
float DynamicWindow::dist(int v, int omega)
{
    float distVOmega = 0, vx, vy, dx, dy;
    float newAngle = m_alpha;
    int minRobotDimension = std::min(m_fishRobotHeight, m_fishRobotWidth);
    QPointF dPos(0,0), newPos;


    for (int i = 1; i<=dtSamples ; i++)
    {
        //! compute new robot orientation
        newAngle += omega*simulation_dt;
        //! compute new translational velocities
        vx =  v*sin(newAngle*DEG2RAD);
        vy = -v*cos(newAngle*DEG2RAD);
        //! compute new position
        dPos.setX(dPos.x()+simulation_dt*vx);
        dPos.setY(dPos.y()+simulation_dt*vy);
        newPos = m_pos+dPos;

        for (int k = newPos.x()-minRobotDimension/2 ; k<=newPos.x()+minRobotDimension/2 ; k++)
        {
            for (int l = newPos.y()-minRobotDimension/2 ; l<=newPos.y()+minRobotDimension/2 ; l++)
            {
                //! if the new position is in bounds
                if(k>=0 && l>=0 && k<m_width && l<m_height)
                {
                    //! variable to avoid conversion issues from QPointF-> QPoint
                    QPoint currentPos(newPos.x(), newPos.y());

                    //! check to see if the configuration space is occupied
                    if(m_configurationSpace.at(k).at(l) == State::OCCUPIED
                       ||robotCloseby(currentPos, 3))
                    {
                        //! compute the total distance travelled along the arc
                        qDebug()<<"start : "<<m_pos.x()<<m_pos.y();
                        qDebug()<<"finish : "<<newPos.x()<<newPos.y();
                        return distVOmega;
                    }

                }
                //! if the new position is outside the configuration space
                else
                {
                    qDebug()<<"PROBLEM";
                    return distVOmega;
                }
            }
        }
        distVOmega += simulation_dt*sqrt(vx*vx+vy*vy);
    }

    qDebug()<<"return"<<distVOmega;
    return distVOmega;
}

//! this method chooses the most optimal linear and angular velocities by using
//! an objective function and choosing the highest scoring combination
std::pair<float,float> DynamicWindow::identifyOptimalVelocities()
{
    std::pair<float,float> optimalVelocities;
    int optimalValue = 0;
    int value;

    for (int i = 0 ; i<(int)m_admissibleVelocities.size(); i++)
    {
        value = computeObjectiveFunction(m_admissibleVelocities.at(i));
        if (value>optimalValue)
        {
            optimalValue = value;
            optimalVelocities = m_admissibleVelocities.at(i);
        }
    }

    return optimalVelocities;
}

//! this method computes the total objective function for a given linear and angular
//! velocity
float DynamicWindow::computeObjectiveFunction(std::pair<float,float> velocity)
{
    float value;

    value = m_alpha*computeAlignFunction()
          + m_beta*computeVelocityFunction(velocity)
          + m_gamma*computeGoalFunction();

    return value;
}

//! this method computes the align objective function for a given linear and angular
//! velocity to insure the robot stays directed towards the goal
float DynamicWindow::computeAlignFunction()
{
    float value, angleToGoal;
    QPoint currentPos = m_fishRobots->at(m_fishRobotId)->getPosition();
    QPointF vObst, vRobot;

    //! Compute angle between obstacle and current robot
    vObst.setX(m_goal.x()-currentPos.x());
    vObst.setY(m_goal.y()-currentPos.y());
    vRobot.setX(sin(m_angle*DEG2RAD));  //! conversion to get radians
    vRobot.setY(-cos(m_angle*DEG2RAD)); //! conversion to get radians
    angleToGoal = (atan2(vObst.y(),vObst.x()) - atan2(vRobot.y(),vRobot.x())); //! in radians

    //! normalize the angle
    if (fabs(angleToGoal) > M_PI)
    {
        angleToGoal-= sgn(angleToGoal)*2*M_PI;
    }

    //! compute the value
    value = 1-fabs(angleToGoal)/M_PI;

    return value;
}

//! this method computes the velocity objective function for a given linear and angular
//! velocity to insure the robot prefers fastforward motion
float DynamicWindow::computeVelocityFunction(std::pair<float,float> velocity)
{
    float value;
    float dist, distLimit;
    QPoint deltaPos;

    //! compute distance to the goal
    deltaPos.setX(m_pos.x() - m_goal.x());
    deltaPos.setY(m_pos.y() - m_goal.y());
    dist = sqrt(deltaPos.x()*deltaPos.x() + deltaPos.y()*deltaPos.y());

    //! compute the limit
    distLimit = std::max(m_width, m_height)/3;

    //! compare to the limit to compute the correct value
    if (dist<distLimit)
    {
        value = velocity.first/m_maxVel;
    }
    else value = 1 - velocity.first/m_maxVel;

    //! return the correct value
    return value;
}

//! this method computes the goal objective function for a given linear and angular
//! velocity to insure the robot keeps going towards the goal
int DynamicWindow::computeGoalFunction()
{
    float dist, distLimit;
    QPoint deltaPos;

    //! compute distance to the goal
    deltaPos.setX(m_pos.x() - m_goal.x());
    deltaPos.setY(m_pos.y() - m_goal.y());
    dist = sqrt(deltaPos.x()*deltaPos.x() + deltaPos.y()*deltaPos.y());

    //! compute the limit
    distLimit = std::max(m_width, m_height)/3;

    //! compare to the limit to compute the correct binary value
    if (dist<distLimit)
    {
        return 1;
    }
    else return 0;
}

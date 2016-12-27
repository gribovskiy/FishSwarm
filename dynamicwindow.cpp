//Autor : Laila El Hamamsy
//Date Created : Sunday December 4th 2016
//Version : 2
//Last Modified : 26.12.2016


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

    //! compute the maximum distance that can be travelled in the config. space
    float maxDist = m_width + m_height;

    //! Initialize the priority planning
    m_priorityPlanning = new PriorityPlanning(m_fishRobots, maxDist);

}


//----------------------------------------------------------------------------//
//--------------------------------Exported Members----------------------------//
//----------------------------------------------------------------------------//

//! Compute the new linear and angular velocities for the given robot if an obstacle is in the vicinity
//! if no obstacle is detected it returns -1,-1 in float, needs to be rounded to int for testing
std::pair<float,float> DynamicWindow::computeNewLinearAndAngularVelIfObstacle(int fishRobotId, QPoint pathGoal)
{
    std::pair<float,float> optimalVel(-1,-1);

    if (fishRobotId == 1)
    {
        return std::make_pair(0,0);
    }

    //! initialize all the important paramters.
    m_fishRobotId   = fishRobotId;
    m_goal          = pathGoal;
    m_pos           = m_fishRobots->at(m_fishRobotId)->getPosition();
    m_vel           = m_fishRobots->at(m_fishRobotId)->getLinearVelocity();
    m_angularVel    = m_fishRobots->at(m_fishRobotId)->getAngularVelocity()*DEG2RAD;
    m_maxVel        = m_fishRobots->at(m_fishRobotId)->getMaxLinearVelocity();
    m_angle         = m_fishRobots->at(m_fishRobotId)->getOrientation()*DEG2RAD;
    m_maxAngularVel = m_fishRobots->at(m_fishRobotId)->getMaxAngularVelocity()*DEG2RAD;

    //! clear the list of admissible velocities
    m_admissibleVelocities.clear();

    //! if no robot is closeby
    if(!robotCloseby(m_pos))
    {
        //! return the base velocities -1,-1
        return optimalVel;
    }

    //! initialize the configuration space for the robots, ie the areas that
    //! are forbidden for the given robot.
    initializeRobotSpace();

    //! if the goal is in the robots' OCCUPIED space get the next point on the
    //! djikstra path
    while (m_robotsSpace.at(m_goal.x()).at(m_goal.y()) == State::OCCUPIED)
    {
        m_goal = m_fishRobots->at(m_fishRobotId)->getNextPathPoint();
        //! If the new target position is the current position, there are no more
        //! path points outside the robots' OCCUPIED space. return v and omega = 0
        if ( m_goal.x() == m_pos.x() && m_goal.y() == m_pos.y())
        {
            return std::make_pair(0,0);
        }
    }

    //! search the space for the admissible velocities, ie the ones that
    //! will help go around the obstacles or stop in time to avoid collision
    //! these admissible tuples will be stored in m_admissible velocities
    searchSpaceForAdmissibleVelocities();

    //! identify out of all the admissible tuples the optimal one using an
    //! objective function.
    optimalVel = identifyOptimalVelocities();


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
    int maxRobotDimension = 2*std::max(m_fishRobotHeight, m_fishRobotWidth); //! FIXME : variable parameter to add to simulator

    //! intialize the robots space
    //! iterate through the configuration space
    for (int i = 0 ; i<m_width; i++)
    {
        for (int j = 0; j<m_height ; j++)
        {
            //! set all coordinates to free
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
            //! get the position of the given robot
            robotPos = m_fishRobots->at(i)->getPosition();

            //! iterate around the robot's position to take into account the
            //! geometry of the robot
            for(int k =  robotPos.x()-maxRobotDimension ; k<=robotPos.x()+maxRobotDimension; k++)
            {
                for(int l =  robotPos.y()-maxRobotDimension ; l<=robotPos.y()+maxRobotDimension; l++)
                {
                    //! if the new position is in bounds
                    if(k>=0 && l>=0 && k<m_width && l<m_height)
                    {
                        //! set the cells around it to occupied
                        m_robotsSpace.at(k).at(l) = State::OCCUPIED;
                    }
                }
            }
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
    int     distLimit = 2*std::max(fishRobotWidth, fishRobotHeight)/distRatio; //! FIXME : variable parameter to add to simulator

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

                //! if the angle is inferior to the limit
                if(fabs(angleToGoal)<M_PI/3)//! FIXME : variable parameter to add to simulator
                {
                    //! return true, there is a robot closeby
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
    float dist;
    std::vector<VELOCITIESDIST> collisionFreeVelocities;
    std::vector<VELOCITIESDIST> admissibleCollisionVelocities;


    //! compute the dynamic frame as the current velocities +/- the acceleration
    //! times the timeframe to take into account acceleration and decceleration
    float minLinearVel  = m_vel-linearAcceleration*m_timeframe;
    float maxLinearVel  = m_vel+linearAcceleration*m_timeframe;
    float minAngularVel = m_angularVel  - angularAcceleration* m_timeframe;
    float maxAngularVel = m_angularVel  + angularAcceleration* m_timeframe;


    //! remove inadmissible velocities from the dynamic window
    minLinearVel  = std::max(minLinearVel,(float)0);
    maxLinearVel  = std::min(maxLinearVel, (float)m_maxLinearVel);
    minAngularVel  = std::max(minAngularVel,-(float)m_maxAngularVel);
    maxAngularVel  = std::min(maxAngularVel, (float)m_maxAngularVel);

    //! compute the dynamic window
    for (int i = 0; i<=numberLinearVel ; i++)
    {
        //! compute the new linear velocity
        float linearVel = minLinearVel
                        + (maxLinearVel - minLinearVel)*i/numberLinearVel;

        //! if the linear velocity is admissible
        if(linearVel<m_maxLinearVel)
        {
            //! test with all combinations of angular velocities
            for(int j = 0; j<=numberAngularVel; j++)
            {
                //! compute the angular velocity
                float angularVel = minAngularVel
                                 + (maxAngularVel-minAngularVel)*j/numberAngularVel;

                //! compute the dsitance that can be travelled given the tuple v omega
                COLLISIONDIST collisionDist = distanceTravelled(linearVel,angularVel);
                dist = collisionDist.distance;

                //! if the velocity lets the robot stop in time to avoid collision
                if (  fabs(linearVel)  <= sqrt(2*dist*linearAcceleration)
                      && fabs(angularVel) <= sqrt(2*dist*angularAcceleration)
                      && (fabs(linearVel)> 0.0001 || fabs(angularVel)>0.0001))
                {
                    //! if no collision has been detected
                    if(!collisionDist.collision)
                    {
                        //! add the tuple to the collision free velocities
                        collisionFreeVelocities.push_back(VELOCITIESDIST(linearVel, angularVel, collisionDist));
                    }
                    else
                    {
                        //! add the tuple to the collision velocities
                        admissibleCollisionVelocities.push_back(VELOCITIESDIST(linearVel, angularVel, collisionDist));
                    }
                }
            }
        }
    }

    //! if there are collision free velocities, prefer those
    if(!collisionFreeVelocities.empty())
    {
        m_admissibleVelocities = collisionFreeVelocities;
    }
    //! else if there are no collision free velocities, take the ones that lead
    //! to collision further along the line
    else if (!admissibleCollisionVelocities.empty())
    {
        m_admissibleVelocities = admissibleCollisionVelocities;
    }
}

//! this method computes the distance travelled within a certain timeframe
//! or the distance to the closest object given a certain velocity
COLLISIONDIST DynamicWindow::distanceTravelled(int v, int omega)
{
    float totalDistance = 0;
    float newAngle = m_alpha; //! in radians
    QPointF dPos(0,0), newPos(m_pos);
    bool collision = false;
    int i = 1;
    COLLISIONDIST collisionDist(collision,totalDistance);

    while(!collision && i<dtSamples)
    {
        //! compute new robot orientation
        newAngle += omega*simulation_dt; //! in radians
        //! normalize the angle
        while (fabs(newAngle) > M_PI)
        {
            newAngle-= sgn(newAngle)*2*M_PI;
        }

        //! compute new position
        dPos.setX(simulation_dt*v*sin(newAngle));
        dPos.setY(-simulation_dt*v*cos(newAngle));
        newPos += dPos;

        //! if the position is in bounds
        if(newPos.x()>=0 && newPos.y()>=0 && newPos.x()<m_width && newPos.y()<m_height)
        {
            //! if the space is occupied by a wall or a robot
            if(m_configurationSpace.at(newPos.x()).at(newPos.y()) == State::OCCUPIED
                    || m_robotsSpace.at(newPos.x()).at(newPos.y()) == State::OCCUPIED)
            {
                //! there is a collision
                collision = true;
            }
        }
        else
        {
            //! else there is no collision
            collision  = true;
        }

        //! compute the new distance only if the space is not occupied
        if (!collision)
        {
            totalDistance +=  sqrt(dPos.x()*dPos.x()+dPos.y()*dPos.y());// simulation_dt*v;
        }
        i++;
    }

    collisionDist.distance = totalDistance;
    collisionDist.collision = collision;

    return collisionDist;
}

//! this method chooses the most optimal linear and angular velocities by using
//! an objective function and choosing the highest scoring combination
std::pair<float,float> DynamicWindow::identifyOptimalVelocities()
{
    std::pair<float,float> optimalVelocity;
    int optimalValue = 0;
    int value;

    //! for all the admissible velocities
    for (int i = 0 ; i<(int)m_admissibleVelocities.size(); i++)
    {
        //! compute the objetive function
        value = computeObjectiveFunction(m_admissibleVelocities.at(i));
        //! if the value is superior to the optimal value
        if (value>optimalValue)
        {
            //! update the optimal value and store the optimum
            optimalValue = value;
            VELOCITIESDIST optimum(m_admissibleVelocities.at(i));
            optimalVelocity = std::make_pair(optimum.linearVel,optimum.angularVel);
        }
    }
    return optimalVelocity;
}

//! this method computes the total objective function for a given linear and angular
//! velocity
float DynamicWindow::computeObjectiveFunction(VELOCITIESDIST velocity)
{
    float value, newAngle,vx,vy;
    QPointF newPos(m_pos), dPos(0,0);

    //! compute new robot orientation
    newAngle = m_angle + velocity.angularVel*simulation_dt; //! in radians

    //! normalize the angle
    while (fabs(newAngle) > M_PI)
    {
        newAngle-= sgn(newAngle)*2*M_PI;
    }

    //! compute new translational velocities
    vx =  velocity.linearVel*sin(newAngle);
    vy = -velocity.linearVel*cos(newAngle);
    //! compute new position
    dPos.setX(simulation_dt*vx);
    dPos.setY(simulation_dt*vy);
    newPos += dPos;

    //! compute the value of the objective function
    value = m_alpha*computeAlignFunction(newPos, newAngle)
          + m_beta *computeVelocityFunction(velocity)
          + m_gamma*computeGoalFunction(newPos)
          + m_delta*computeDistanceFunction(velocity);

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
    return value;
}

//! this method computes the velocity objective function for a given linear and angular
//! velocity to insure the robot prefers fastforward motion
float DynamicWindow::computeVelocityFunction(VELOCITIESDIST velocity)
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
        value = velocity.linearVel/m_maxVel;
    }
    else value = 1 - velocity.linearVel/m_maxVel;

    //! return the correct value
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
    distLimit = std::max(m_width, m_height)/5; //! FIXME : variable parameter to be added to simulator

    //! compare to the limit to compute the correct binary value
    if (dist<distLimit)
    {
        return 1;
    }

    return 0;
}

//! this method computes the distance objective function for a given localization
//! and velocity. It returns the longest distance the robot can go without
//! collision with respect to the total distance it can travel within the given timeframe
float DynamicWindow::computeDistanceFunction(VELOCITIESDIST velocity)
{
    float distance = 0;
    QPointF dPos(0,0), newPos(m_pos);

    //! Convert it to a value between 0 and 1
    if (fabs(velocity.linearVel) < 0.00001)
    {
        //! if the distance is close to 0, return 0
        distance = 0;
    }
    else
    {
        //! take the distance travelled and divide it by the distance that could
        //! have been travelled within the timeframe given the linear velocity
        distance = velocity.collisionDistance.distance/(velocity.linearVel*m_timeframe);
    }

    return distance;
}

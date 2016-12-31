//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 5
//Last Modified : 30/12/2016
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#include "fishrobot.h"

//----------------------------------------------------------------------------//
//-------------------------------Class Constructors---------------------------//
//----------------------------------------------------------------------------//

/*!
 * Class constructor. Instanciates the fishRobot with its target and ID
 * and sets the intial orientation
 */
FishRobot::FishRobot(Target *targetptr, int fishRobotID) : m_angle(315)
{
    //! associate to each fish robot a specific target
    m_target = targetptr;

    //! associate to each fish robot a specific ID
    m_fishRobotID = fishRobotID;
}


//----------------------------------------------------------------------------//
//------------------------- QGraphics Item Class Methods ---------------------//
//----------------------------------------------------------------------------//

/*!
 * QGraphicsItem method, it returns the estimated area for the Target drawing
 */
QRectF FishRobot::boundingRect() const
{
    //! part of the qGraphicsItem instanciation
    qreal adjust = 2;

    //! return the bounding rectangle
    return QRectF(-m_fishRobotWidth/2-adjust,-m_fishRobotHeight/2-adjust,
                      m_fishRobotWidth+adjust, m_fishRobotHeight+adjust);
}

/*!
 * QGraphicsItem method, it returns returns the shape of the fishRobot
 */
QPainterPath FishRobot::shape() const
{
    //! same : part of the qGraphicsItem instanciation
    QPainterPath path;

    //! return the path for the bounding rectangle
    path.addRect(20,20,40,40);
    return path;
}

/*!
 * QGraphicsItem method, paints the fishRobot
 */
void FishRobot::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    //! Draw the Tail
    painter->setBrush(QColor(0, 0, 255, 127));
    static QPointF points[3] = {QPointF(0, +m_fishRobotHeight/4),
                                QPointF(-m_fishRobotWidth/2, +m_fishRobotHeight/2),
                                QPointF(+m_fishRobotWidth/2, +m_fishRobotHeight/2)};
    painter->drawPolygon(points,3);

    //! Draw the Body
    painter->setBrush(QColor(0, 0, 255, 127));
    QRectF body(-m_fishRobotWidth/2, -m_fishRobotHeight/2, m_fishRobotWidth, m_fishRobotHeight*4/5);
    painter->drawEllipse(body);

    //! Draw the Eyes
    painter->setBrush(scene()->collidingItems(this).isEmpty() ? Qt::white : Qt::darkRed);
    QRectF leftEye(-m_fishRobotWidth/2, -m_fishRobotHeight/2, m_fishRobotWidth/2, m_fishRobotHeight/4);
    QRectF rightEye(0, -m_fishRobotHeight/2, m_fishRobotWidth/2, m_fishRobotHeight/4);
    painter->drawEllipse(leftEye);
    painter->drawEllipse(rightEye);
}

//----------------------------------------------------------------------------//
//----------------------------- Non Exported Members -------------------------//
//----------------------------------------------------------------------------//

//---------------------------------------//
//------------Advance Methods------------//
//---------------------------------------//

/*!
 * Non Exported Member.this method gives the following position for the
 * robot using simple PID controller to follow the target
 */
void FishRobot::advancePID()
{
    //! the goal is the target
    computeNewVelocitiesAndNewPosition(m_target->getPosition());
}

/*!
 * Non Exported Member. this method gives the following position for the
 * robot using Djikstra to follow the target
 */
void FishRobot::advanceDjikstra()
{
    QPoint goalCoord = identifyDjikstraTarget();

    if (goalCoord.x()==-1 && goalCoord.y() == -1)
    {
        return;
    }

    //! if the robot has not reached the final target, compute new
    //! velocities and position
    computeNewVelocitiesAndNewPosition(goalCoord);
}

/*!
 * Non Exported Member. this method gives the following position for the
 * robot using Djikstra with DWA as obstacle avoidace to follow the target
 */
void FishRobot::advanceDjikstraDWA()
{
    QPoint goalCoord = identifyDjikstraTarget();
    float distToGoal = computeDistance(goalCoord,m_position);

    //! if the dijkstra path has not been set and the target is the final
    //! target
    if(m_dijkstraPath.empty() && distToGoal >2*m_targetDist)
        return;

    advanceDjikstra();
}

/*!
 * Non Exported Member.this method gives the following position for the
 * robot using Potential Field to follow the target
 */
void FishRobot::advancePotField(QPoint goalCoord, int targetDist)
{
    std::pair<float,float> force;
    float distFinalTarget, forceNorm;
    QPoint intermediateTargetCoord;

    //! compute the distance to the final target
    distFinalTarget = sqrt((m_target->getPosition().x()-m_position.x())
                           *(m_target->getPosition().x()-m_position.x())
                           +(m_target->getPosition().y()-m_position.y())
                           *(m_target->getPosition().y()-m_position.y()));

    //! if the distance to the final target is inferior to the admissible distance
    if(distFinalTarget<m_targetDist)
    {
        //! don't do anything
        return;
    }

    //! compute the potential field force given the goal coordinates
    force = m_potentialField->computeTotalForceForRobot(m_fishRobotID, goalCoord);

    //! compute the intermediate goal
    intermediateTargetCoord.setX(m_position.x()+force.first);
    intermediateTargetCoord.setY(m_position.y()+force.second);

    /*
    //! VARIANT 1
    //! set the linear velocity to the desired linear velocity
    m_linearVel = m_desiredLinearVel;
    */

    //! VARIANT 2
    //! compute the linear velocity
    m_linearVel = sqrt(force.first*force.first+force.second*force.second);

    //! If the linear velocity is superior to the max linearVel specified
    if (m_linearVel > m_desiredLinearVel)
    {
        //! Scale the force down
        force.first  = force.first*m_desiredLinearVel/m_linearVel;
        force.second = force.second*m_desiredLinearVel/m_linearVel;
        m_linearVel = m_desiredLinearVel;
    }


    //! compute the new angular velocity given the intermediate target
    m_omega = computeAngularVelocity(intermediateTargetCoord); //! in degrees

    //! compute the new position and new orientation
    computeNewPositionAndOrientation();
}

/*!
 * Non Exported Member. this method gives the following position for the
 * robot using Djikstra and potential field to follow the target
 */
void FishRobot::advanceDjikstraPotField()
{
    QPoint goalCoord = identifyDjikstraTarget();
    float distToGoal = computeDistance(goalCoord,m_position);

    //! if the dijkstra path has not been set and the target is the final
    //! target
    if(m_dijkstraPath.empty() && distToGoal >2*m_targetDist)
    {
         m_status = FishBotStatus::NOTARGET;
         return;
    }

    if (goalCoord.x() == -1 && goalCoord.y() == -1)
    {
        return;
    }

    int targetDist;

    if (m_dijkstraPath.size()>1)
    {
        targetDist = m_intermediateTargetDist;
    }
    else targetDist = m_targetDist;

    advancePotField(goalCoord, targetDist);
}

/*!
 * Exported Member. This QGraphics method handles the animation and computes
 * the movement of each fishRobot at each timestep using the different
 * path planning and obstacle avoidance methods
 */
void FishRobot::advance(int step = 1)//! moves each fish at each step of the program
{
    //! if a timestep has elapsed
    if (!step)
    {
        return;
    }

    if(m_pathplanning == PathPlanning::DIJKSTRADWA && m_fishRobotID == 1)
    {
        m_position.setX(500);
        m_position.setY(530);
        setPos(m_position);
        setRotation(300);
        return;
    }

    //!place the fish robots and their targets for demonstration purposes
    placeFishRobotsAndTargets();

    //! test if at target, if the case return
    float distanceToFinalTarget = computeDistance(m_position, m_target->getPosition());
    if (distanceToFinalTarget<m_targetDist)
    {
        m_status = FishBotStatus::TARGET_REACHED;
        return;
    }

    //! call the correct advance method given the chosen path planning method
    switch(m_pathplanning)
    {
    case PathPlanning::PID :
                advancePID();
                break;
    case PathPlanning::DIJKSTRA :
                advanceDjikstra();
                break;
    case PathPlanning::DIJKSTRADWA :
                advanceDjikstraDWA();
                break;
    case PathPlanning::POTFIELD :
                advancePotField(m_target->getPosition(), m_targetDist);
                break;
    case PathPlanning::DIJKSTRAPOTFIELD:
                advanceDjikstraPotField();
                break;
    }
}

/*!
 * Non Exported Member. This method positions the fishRobots and the targets,
 * for Demonstration and evaluation purposes only
 */
void FishRobot::placeFishRobotsAndTargets()
{
    QPoint point(100, 150);

    //! set the targets the fishRobots
    if (m_fishRobotID == 0 || m_fishRobotID == 1 || m_fishRobotID == 2)
    {
        point.setY(point.y());
        m_target->setPosition(point);
    }


    /*
    //! Reinitialize the fishRobots' position if the final target
    //! has been reached, for demonstration purposes
    if ((m_pathplanning == PathPlanning::DIJKSTRADWA || m_pathplanning == PathPlanning::DIJKSTRA)
       && m_dijkstraPath.empty())
    {
        if (m_fishRobotID == 0)
        {
            QPoint point(550, 700);
            m_position = point;
            m_angle = 0;
        }
        if (m_fishRobotID == 1)
        {
            QPoint point(700, 700);
            m_position = point;
            m_angle = 90;
        }
        if (m_fishRobotID == 2)
        {
            QPoint point(700, 500);
            m_position = point;
            m_angle = 0;
        }

        setPos(m_position);
        return;
    }
    */
}


/*!
* Non Exported Member. This method identifies the closest point in the
* Dijkstra path to avoid backtracking.It deletes all points preceding the
* identified one This method is adapted to the case where all the dijkstra
* path points are available, not just the reduced path
*/

//! TODO :2 functions instead of one
void FishRobot::eliminateBackwardsDijkstraPathPoints()
{
    if (m_dijkstraPathType == DijkstraPath::COMPLETE)
    {
        float deltaPos;
        float mindist = m_simulationWidth + m_simulationHeight;
        int index = 0;
        //! for all the path points
        for (auto i = m_dijkstraPath.begin(); i!= m_dijkstraPath.end();i++)
        {
            //! compute the distance between the current position and the path point
            deltaPos =  computeDistance(m_position, *i);

            //! if the computed distance is inferior to the min distance
            if (deltaPos<mindist)
            {
                //! update minimun distance and the index of the minimun distance
                mindist = deltaPos;
                index = std::distance(m_dijkstraPath.begin(),i);
            }
        }
        //! erase all the path points that are prior to the closest point
        m_dijkstraPath.erase(m_dijkstraPath.begin(), m_dijkstraPath.begin()+index);
    }
}

/*!
 * Non Exported Member. this method gives the following target position for the
 * robot using Djikstra. It returns -1,-1 if there is no target.
 */
QPoint FishRobot::identifyDjikstraTarget()
{
    float   distGoal;
    QPoint  deltaCoord, goalCoord;

    //! If the dijkstra path is not empty
    if (!m_dijkstraPath.empty())
    {
        //! find point in path with minimal distance from the fish
        eliminateBackwardsDijkstraPathPoints();
        goalCoord = m_dijkstraPath.at(0);
    }

    //! Difference of coordinates between current position and goal
    deltaCoord.setX(goalCoord.x()-m_position.x());
    deltaCoord.setY(goalCoord.y()-m_position.y());

    //! Distance to Goal
    distGoal = sqrt(pow((deltaCoord.x()),2) + pow(deltaCoord.y(),2));

    //! if the path is not empty and the distance to the goal is inferior to
    //! the admissible distance
    if (!m_dijkstraPath.empty() && distGoal<m_intermediateTargetDist)
    {
        //! remove the first element from the path
        m_dijkstraPath.erase(m_dijkstraPath.begin());
        //! if the path is not empty
        if (!m_dijkstraPath.empty())
        {
            //! the new goal is the first point in the path
            goalCoord = m_dijkstraPath.at(0);
        }
    }
    //! if the path is empty and the distance to the goal is inferior
    //! to the admissible distance to the target
    if (m_dijkstraPath.empty() && distGoal<m_targetDist)
    {
        m_status = FishBotStatus::TARGET_REACHED;
        //! the robot has reached the final target don't do anything
       goalCoord.setX(-1);
       goalCoord.setY(-1);
    }
    return goalCoord;
}

//---------------------------------------------//
//-Methods to Compute Velocities and Positions-//
//---------------------------------------------//

/*!
 * Non Exported Member.this method computes the new linear and angular
 * velocities for the robot given the goal coordinates
 */
void FishRobot::computeNewVelocitiesAndNewPosition(QPoint goalCoord)
{
    float   distGoal;
    QPoint  deltaCoord, vGoal;
    std::pair<float, float> velocities;

    //! Difference of coordinates between current position and goal
    deltaCoord.setX(goalCoord.x()-m_position.x());
    deltaCoord.setY(goalCoord.y()-m_position.y());

    //! Distance to Goal
    distGoal = sqrt(pow((deltaCoord.x()),2) + pow(deltaCoord.y(),2));

    //! if we are close enough to the target
    if (distGoal<5)
    {
        m_status = FishBotStatus::TARGET_REACHED;
        return;
    }


    //! if we are using djikstra with DWA obstacle avoidance to avoid collision with mobile obstacles
    if (m_pathplanning == PathPlanning::DIJKSTRADWA)
    {
         //! compute the dynamic window optimal velocities
         velocities = m_dynamicWindow->computeNewLinearAndAngularVelIfObstacle(m_fishRobotID, goalCoord);

         //! convert the velocities to integer with rounding
         int velocityLin = (velocities.first-0.5);
         int velocityAng = (velocities.second-0.5);

         //! If the velocities are inadmissible, ie no robot has been detected
         //! closeby
         if(velocityLin == -1 && velocityAng == -1)
         {
             //! set the linear velocity to the desired velocity
             m_linearVel = m_desiredLinearVel;
             //! compute the angular velocity using PID controller
             m_omega     = computeAngularVelocity(goalCoord);

         }
         else
         {
             //! if the velocities are admissible, ie a robot has been detected
             //! closeby set the velocities to the dynamic window output
             m_linearVel  = velocities.first;
             m_omega    = velocities.second*RAD2DEG;
         }
    }
    else
    {
        //! if we are not using dijkstra with DWA set linear velocity to desired
        //! velocity and compute angular velocity
         m_linearVel = m_desiredLinearVel;
         m_omega     = computeAngularVelocity(goalCoord);
    }

    //! compute the new position and orientation of the robot given the velocites
    computeNewPositionAndOrientation();
}

/*!
 * Non Exported Member. this method computes the new position and rotation
 * once the linear velocity and angular velocity have been set.
 */
void FishRobot::computeNewPositionAndOrientation()
{
    //! New FishRobot angle
    m_angle += m_omega*simulation_dt;

    //! Normalize the angle
    while (fabs(m_angle) > 180)
    {
        m_angle-= sgn(m_angle)*360;
    }

    //! UNICYCLE MODEL IMPLEMENTATION
    m_vx =  m_linearVel*sin(m_angle*DEG2RAD);
    m_vy = -m_linearVel*cos(m_angle*DEG2RAD);

    if(m_vx == 0 && m_vy == 0)
    {
        m_status = FishBotStatus::BLOCKED;
    }
    else
    {
         m_status = FishBotStatus::MOVING;
    }

    //! New FishRobot Position
    m_position.setX(m_position.x()+m_vx*simulation_dt);
    m_position.setY(m_position.y()+m_vy*simulation_dt);

    //! Inverse Kinematics
    m_vl = m_linearVel + m_omega*DIST_WHEELS/2.0; //normally divided by the radius of the wheels
    m_vr = m_linearVel - m_omega*DIST_WHEELS/2.0; //same

    //! set position and orientation
    setRotation(m_angle);
    setPos(m_position);
}

/*!
 * Non Exported Member. this method computes the angular velocity given
 * the goalCoordinates by determining the angle to the goal and calling
 * the PID controller
 */
float FishRobot::computeAngularVelocity(QPoint goalCoord)
{
    float omega, alphaGoal;
    QPointF vFish, vGoal;

    //! compute angle to Goal
    vGoal.setX(goalCoord.x()-m_position.x());
    vGoal.setY(goalCoord.y()-m_position.y());
    vFish.setX(sin(m_angle*DEG2RAD));  //convert to radians
    vFish.setY(-cos(m_angle*DEG2RAD)); //convert to radians

    alphaGoal = (atan2(vGoal.y(),vGoal.x()) - atan2(vFish.y(),vFish.x()))*RAD2DEG;

    //! normalize the angle to the goal
    while (fabs(alphaGoal) > 180)
    {
        alphaGoal-= sgn(alphaGoal)*360;
    }

    //! Constant Tangential Velocity PID controller (cf. wheeledRobots.cpp)
    omega = pidController(goalCoord, alphaGoal)*RAD2DEG; //passage en degres

    //! if the angular speed is superior to the max speed, set omega to omega
    //! max with respect to its sign.
    if (abs(omega)>m_omegaMax)
    {
        omega = sgn(omega)*m_omegaMax;
    }

    //! return the angular velocity
    return omega;
}

/*!
 * Non Exported Member. This method implements a PID controller on the
 * angular velocity
 * //!NOTE : this method was retrieved from CATS and readapted to this class
 */
float FishRobot::pidController(QPoint goalCoord, float alphaGoal)
{
    std::vector<double> tmp(2), PosD(2), Pos(2), T(2);
    static std::vector<double> error_PID(5), error_buffer;
    float rho = 0, alpha = 0, AngVel = 0;

    Pos.at(0) = m_position.x();
    Pos.at(1) = m_position.y();
    PosD.at(0) = goalCoord.x();
    PosD.at(1) = goalCoord.y();

    /// PID controller assuming a constant tangential speed
    /// position error calculation
    tmp.at(0) = PosD.at(0) - Pos.at(0);
    tmp.at(1) = PosD.at(1) - Pos.at(1);

    /// Error represented into the robot's coordinate system

    /// T.at(0,0) = cos(Pos.at(2));   T.at(0,1) = sin(Pos.at(2));
    /// T.at(1,0) =-sin(Pos.at(2));   T.at(1,1) = cos(Pos.at(2));

    T.at(0) = tmp.at(0)*cos(m_angle*DEG2RAD) + tmp.at(1)*sin(m_angle*DEG2RAD);
    T.at(1) = -tmp.at(0)*sin(m_angle*DEG2RAD) + tmp.at(1)*cos(m_angle*DEG2RAD);

    tmp = T;

    rho   = sqrt(tmp.at(0)*tmp.at(0) + tmp.at(1)*tmp.at(1));
    alpha = alphaGoal;

    error_buffer.push_back(alpha);
    error_PID.at(0) = alpha;
    error_PID.at(1) = rho;
    error_PID.at(2) += error_buffer.back() - error_buffer.front(); // - dummy; remplacé
    error_PID.at(3) = (alpha - error_PID.at(4))/30.;    // Assuming 30 FPS
    error_PID.at(4) = alpha;

    //qDebug()<<"alphaGoal : "<<alphaGoal<<" rho :"<<rho;

    if (error_PID.at(0) < 1e-1 && error_PID.at(1) > 1.0) {
        AngVel = 0.0;
    } else {
        AngVel = m_Kp * error_PID.at(0) + m_Ki * error_PID.at(2) - m_Kd * error_PID.at(3);
    }
    AngVel = m_Kp * error_PID.at(0) + m_Ki * error_PID.at(2) - m_Kd * error_PID.at(3);
    if (abs(AngVel)>m_omegaMax)
    {
        AngVel = sgn(AngVel)*m_omegaMax;
    }

    //qDebug()<<"Kp : "<<m_Kp<< "|| Ki : "<<m_Ki<<" || Kd : "<<m_Kd;
    //qDebug()<<"omega : "<<AngVel;

    return AngVel;
}

//! this method computes the distance between 2 points.
float FishRobot::computeDistance(QPoint pos1, QPoint pos2)
{
    float dX = pos1.x()-pos2.x();
    float dY = pos1.y()-pos2.y();

    return sqrt(dX*dX + dY*dY);
}

//----------------------------------------------------------------------------//
//------------------------------- Setter Methods -----------------------------//
//----------------------------------------------------------------------------//

/*!
 * Exported Member. This method sets the new ID for the fishRobot
 */
void FishRobot::setFishRobotID(int fishRobotID)
{
    m_fishRobotID = fishRobotID;
}

/*!
 * Exported Member. This method sets the new position for the fishRobot
 */
void FishRobot::setPosition(QPoint newPosition)
{
    m_position = newPosition;
}

/*!
 * Exported Member. This method sets the new PID controller paramters
 * It receives as input the type of gain as well as the new value.
 */
void FishRobot::setControllerParameters(Gains gain, double newK)
{
    switch(gain){
        case Gains::PROP :
                m_Kp = newK;
            break;
        case Gains::INTEG :
                m_Ki = newK;
            break;
        case Gains::DERIV :
                m_Kd = newK;
            break;
    }
}

/*!
 * Exported Member. This method sets the maximum angular velocity
 */
void FishRobot::setOmegaMax(int newOmegaMax)
{
    m_omegaMax = newOmegaMax;
}

/*!
 * Exported Member. This method sets the new desired linear velocity
 */
void FishRobot::setDesiredLinearVel(int newLinearVel)
{
    m_desiredLinearVel = newLinearVel;
}

/*!
 * Exported Member. This method sets the new fish robot dimensions (width
 * and height)
 */
void FishRobot::setFishRobotDimensions(float newRobotWidth, float newRobotHeight)
{
    m_fishRobotWidth = newRobotWidth;
    m_fishRobotHeight = newRobotHeight;
}

/*!
 * Exported Member. This method sets the new dijkstra path.
 */
void FishRobot::setDijkstraPath(std::vector<QPoint> newPath)
{
    m_dijkstraPath.clear();
    m_dijkstraPath = newPath;
}

/*!
 * Exported Member.this method gives the fishRobot access to the potential
 * field in order to determine the velocities at the next step
 */
void FishRobot::setPotentialField(PotentialField* newPotField)
{
    m_potentialField = newPotField;
}

/*!
 * Exported Member. this method gives the fishRobot access to the
 * dynamic window in order to modify trajectory if an obstacle is
 * detected on the current path
 */
void FishRobot::setDynamicWindow(DynamicWindow* newDynamicWindow)
{
    m_dynamicWindow = newDynamicWindow;
}

/*!
 * Exported Member. this method helps determine which path planning method
 * should be used for the robots
 */
void FishRobot::setPathPlanningMethod(PathPlanning newPathPlanning)
{
    m_pathplanning = newPathPlanning;
}

/*!
 * Exported Member. this method helps sets the admissible distances to the
 * intermediate and final target.
 */
void FishRobot::setAdmissibleTargetDistances(int intermediateTargetDist, int finalTargetDist)
{
    m_intermediateTargetDist = intermediateTargetDist;
    m_targetDist = finalTargetDist;
}

/*!
 * Exported Member. this method stores the chosen dijkstra path type in the
 * simulator. Can be modified by directly incorporation dijkstra to the
 * fishrobot as was done for potential field and dynamic window.
 */

void FishRobot::setDijkstraPathType(DijkstraPath pathType)
{
    m_dijkstraPathType = pathType;
}

//----------------------------------------------------------------------------//
//------------------------------- Getter Methods -----------------------------//
//----------------------------------------------------------------------------//

/*!
 * Exported Member. This method gets the fishRobot's position
 */
QPoint FishRobot::getPosition()
{
    return m_position;
}

/*!
 * Exported Member. This methods returns the orientation of the fish robot
 */
float FishRobot::getOrientationDeg()
{
    return m_angle;
}

/*!
 * Exported Member. This method gets the final target's position unless the
 * path planning method is the dijkstra + DWA in which case it gives the
 * position of the next target
 */
QPoint FishRobot::getTargetPosition()
{
    return m_target->getPosition();
}

/*!
 * Exported Member.this method returns the current linear velocity of the
 * fishRobot.
 */
int FishRobot::getLinearVelocity()
{
    return m_linearVel;
}

/*!
 * Exported Member.this method returns the current angular velocity of the
 * fishRobot.
 */
int FishRobot::getAngularVelocity()
{
    return m_omega;
}

/*!
 * Exported Member.this method returns the maximum linear velocity for the
 * fishRobot.
 */
int FishRobot::getMaxLinearVelocity()
{
    return m_maxLinearVel;
}

/*!
 * Exported Member. this method returns the maximum angular velocity for
 * the fishRobot.
 */
int FishRobot::getMaxAngularVelocity()
{
    return m_omegaMax;
}

/*!
 * Exported Member. This method returns the fishRobot's width
 */
int FishRobot::getFishRobotWidth()
{
    return m_fishRobotWidth;
}

/*!
 * Exported Member. This method returns the fishRobot's height
 */
int FishRobot::getFishRobotHeight()
{
    return m_fishRobotHeight;
}

/*!
 * Exported Member.this method will get the next target given a djikstra
 * path, if there are no other path points it will return the current
 * position.
 */
QPoint FishRobot::getNextPathPoint()
{
    //! remove the current path point
    m_dijkstraPath.erase(m_dijkstraPath.begin());

    //! if there are no other path points
    if (m_dijkstraPath.empty())
    {
        //! return the current position of the robot as the next target
        return m_position;
    }
    else
    {
        //! return the new current target
        return m_dijkstraPath.at(0);
    }
}


/*!
 * Exported Member.this method returns the path computed by dijkstra's
 * shortest path algorithm for the given fish robot
 */
std::vector<QPoint> FishRobot::getDijkstraPath()
{
    return m_dijkstraPath;
}

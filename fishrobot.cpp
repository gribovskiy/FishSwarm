//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 4
//Last Modified :
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#include "fishrobot.h"

//----------------------------------------------------------------------------//
//-------------------------------Class Constructors---------------------------//
//----------------------------------------------------------------------------//

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

QRectF FishRobot::boundingRect() const
{
    //! part of the qGraphicsItem instanciation
    qreal adjust = 2;

    return QRectF(-m_fishRobotWidth/2-adjust,-m_fishRobotHeight/2-adjust,
                      m_fishRobotWidth+adjust, m_fishRobotHeight+adjust);
}

QPainterPath FishRobot::shape() const
{
    //same : part of the qGraphicsItem instanciation
    QPainterPath path;
    path.addRect(-10, -20, 20, 40); //CHECK IF VALUES DO NOT NEED TO BE MODIFIED
    return path;
}

void FishRobot::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    // Tail
    painter->setBrush(QColor(0, 0, 255, 127));
    static QPointF points[3] = {QPointF(0, +m_fishRobotHeight/4),
                                QPointF(-m_fishRobotWidth/2, +m_fishRobotHeight/2),
                                QPointF(+m_fishRobotWidth/2, +m_fishRobotHeight/2)};
    painter->drawPolygon(points,3);

    // Body
    painter->setBrush(QColor(0, 0, 255, 127));
    QRectF body(-m_fishRobotWidth/2, -m_fishRobotHeight/2, m_fishRobotWidth, m_fishRobotHeight*4/5);
    painter->drawEllipse(body);

    // Eyes
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

//! this method gives the following position for the robot using simple PID controller to follow the target
void FishRobot::advancePID()
{
    QPoint  goalCoord = m_target->getPosition();
    computeNewVelocitiesAndNewPosition(goalCoord);
}

//! this method gives the following position for the robot using Djikstra to follow the target
void FishRobot::advanceDjikstra()
{
    float   distGoal;
    QPoint  deltaCoord, goalCoord;

    if (m_path.empty())
    {
        if (m_fishRobotID == 0)
        {
            QPoint point(550, 700);
            m_position = point;
        }
        if (m_fishRobotID == 1)
        {
            QPoint point(700, 700);
            m_position = point;
        }
        if (m_fishRobotID == 2)
        {
            QPoint point(700, 500);
            m_position = point;
        }

        setPos(m_position);
        return;
    }
    else
    {
        //! dfind point in path with minimal distance from the fish
        identifyClosestPathPoint();
        goalCoord = m_path.at(0);
    }

    //! Difference of coordinates between current position and goal
    deltaCoord.setX(goalCoord.x()-m_position.x());
    deltaCoord.setY(goalCoord.y()-m_position.y());

    //! Distance to Goal
    distGoal = sqrt(pow((deltaCoord.x()),2) + pow(deltaCoord.y(),2));

    if (!m_path.empty() && distGoal<25)
    {
        m_path.erase(m_path.begin());
        //! change the goal
        if (!m_path.empty())
        {
            goalCoord = m_path.at(0);
        }
        else
        {
            return;
        }
    }
    else if (m_path.empty() && distGoal<5)
    {
       return;
    }

    computeNewVelocitiesAndNewPosition(goalCoord);
}

//! This method identifies the closest point in the path and makes it the next
//! target to avoid unecessary detours
void FishRobot::identifyClosestPathPoint()
{
    float mindist = std::max(simulationWidth, simulationHeight);
    float distance;
    int index = 0;

    for (auto i = m_path.begin(); i!= m_path.end();i++)
    {
        distance = pow((m_position.x() - i->x()),2) + pow((m_position.y() - i->y()),2);
        if (distance<mindist)
        {
                mindist = distance;
                index = std::distance(m_path.begin(),i);
        }
    }

    m_path.erase(m_path.begin(), m_path.begin()+index);
}

//! this method gives the following position for the robot using Potential Field to follow the target
void FishRobot::advancePotField()
{
    std::pair<float,float> force = m_potentialField->computeTotalForceForRobot(m_fishRobotID);
    float  forceNorm  = sqrt(force.first*force.first + force.second*force.second); //TODO : useful or not?
    float  forceAngle = atan2(force.second, force.first)*RAD2DEG;
    QPoint goalCoord(m_position.x()+force.first, m_position.y()+force.second);


    qDebug()<<"pseudo target"<<goalCoord.x()-m_position.x()<<goalCoord.y()-m_position.y();
    //! Compute linear Velocity as the norm of the force

    /*
    m_linearVel = sqrt(force.first*force.first+force.second*force.second);

    //! If the linear velocity is superior to the max linearVel specified
    if (m_linearVel > m_desiredLinearVel)
    {
        //! Scale the force down
        force.first  = force.first*m_desiredLinearVel/m_linearVel;
        force.second = force.second*m_desiredLinearVel/m_linearVel;
        m_linearVel = m_desiredLinearVel;
    }
    */

    m_linearVel = m_desiredLinearVel;
    m_omega = computeAngularVelocity(goalCoord);//in degrees

    //! New FishRobot angle
    m_angle += m_omega*simulation_dt;

    //! normalize the angle
    while (fabs(m_angle) > 180)
    {
        m_angle-= sgn(m_angle)*360;
    }



    //! UNICYCLE MODEL IMPLEMENTATION
    m_vx =  m_linearVel*sin(m_angle*DEG2RAD);
    m_vy = -m_linearVel*cos(m_angle*DEG2RAD);

    //! New FishRobot Position
    m_position.setX(m_position.x()+m_vx*simulation_dt);
    m_position.setY(m_position.y()+m_vy*simulation_dt);

    //! Inverse Kinematics
    m_vl = m_linearVel + m_omega*DIST_WHEELS/2.0; //normalement divisé par le rayon des roues..
    m_vr = m_linearVel - m_omega*DIST_WHEELS/2.0; // idem

    setPos(m_position);
    setRotation(m_angle);
}

void FishRobot::advance(int step = 1)//! moves each fish at each step of the program
{
    if (!step)
    {
        return;
    }

    if(m_fishRobotID ==1 && m_pathplanning == PathPlanning::DJIKSTRADWA)
    {
        m_position.setX(500);
        m_position.setY(530);
        setPos(m_position);
        m_angle = 0;
        setRotation(m_angle);
    }

    if (m_fishRobotID == 0)
    {
        QPoint point(50, 200);
        m_target->setPosition(point);
    }
    if (m_fishRobotID == 1)
    {
        QPoint point(200, 50);
        m_target->setPosition(point);
    }
    if (m_fishRobotID == 2)
    {
        QPoint point(100, 100);
        m_target->setPosition(point);
    }


    switch(m_pathplanning)
    {
    case PathPlanning::PID :
                advancePID();
                break;
    case PathPlanning::DJIKSTRA :
                advanceDjikstra();
                break;
    case PathPlanning::DJIKSTRADWA :
                advanceDjikstra();
                break;
    case PathPlanning::POTFIELD :
                advancePotField();
                break;
    }
}


//---------------------------------------//
//----Methods to Compute Velocities------//
//---------------------------------------//

//! this method computes the new linear velocities for the robot given the goal coordinates
void FishRobot::computeNewVelocitiesAndNewPosition(QPoint goalCoord)
{
    float   distGoal;
    QPoint  deltaCoord, vGoal;
    std::pair<float, float> velocities;

    //Difference of coordinates between current position and goal
    deltaCoord.setX(goalCoord.x()-m_position.x());
    deltaCoord.setY(goalCoord.y()-m_position.y());

    //Distance to Goal
    distGoal = sqrt(pow((deltaCoord.x()),2) + pow(deltaCoord.y(),2));

     if(m_fishRobotID == 1 && m_pathplanning == PathPlanning::DJIKSTRADWA)
     {
         return;
     }

    //! if we are close enough to the target
    if (distGoal<5)
    {
       return;
    }
    if(distGoal<10 && m_pathplanning == PathPlanning::POTFIELD)
    {
        return;
    }

    //! if we are using djikstra with DWA obstacle avoidance to avoid collision with mobile obstacles
    if (m_pathplanning == PathPlanning::DJIKSTRADWA)
    {
         velocities = m_dynamicWindow->computeNewLinearAndAngularVelIfObstacle(m_fishRobotID, goalCoord);

         int velocityLin = (velocities.first-0.5);
         int velocityAng = (velocities.second-0.5);

         if(velocityLin == -1 && velocityAng == -1)
         {

             m_linearVel = m_desiredLinearVel;
             m_omega     = computeAngularVelocity(goalCoord);
            // qDebug()<<"FISH NO OUTPUT: m_linearVel : "<<m_linearVel;
         }
         else
         {
             m_linearVel  = velocities.first;
             m_omega    = velocities.second*RAD2DEG;
            // qDebug()<<"FISH OUTPUT: velocity first: "<<m_linearVel;
         }

    }
    else
    {
         m_linearVel = m_desiredLinearVel;
         m_omega     = computeAngularVelocity(goalCoord);
    }

   // qDebug()<<"FISH: m_linearVel : "<<m_linearVel;
   // qDebug()<<"FISH: m_desiredVel : "<<m_desiredLinearVel;

    //if(m_linearVel>m_maxLinearVel)
    {
       // qDebug()<<"PROBLEM";
        //for(long i = 0 ; i<64000 ; i++)
        {

        }
    }

    //! New FishRobot angle
    m_angle += m_omega*simulation_dt;

    //! UNICYCLE MODEL IMPLEMENTATION
    m_vx =  m_linearVel*sin(m_angle*DEG2RAD);
    m_vy = -m_linearVel*cos(m_angle*DEG2RAD);

    //! New FishRobot Position
    m_position.setX(m_position.x()+m_vx*simulation_dt);
    m_position.setY(m_position.y()+m_vy*simulation_dt);

    //! Inverse Kinematics
    m_vl = m_linearVel + m_omega*DIST_WHEELS/2.0; //normalement divisé par le rayon des roues..
    m_vr = m_linearVel - m_omega*DIST_WHEELS/2.0; // idem

    setRotation(m_angle);
    setPos(m_position);
}

float FishRobot::computeAngularVelocity(QPoint goalCoord)
{
    float omega, alphaGoal;
    QPointF vFish, vGoal;

    //angle to Goal (unecessary variables but easier to read)
    vGoal.setX(goalCoord.x()-m_position.x());
    vGoal.setY(goalCoord.y()-m_position.y());
    vFish.setX(sin(m_angle*DEG2RAD));  //pour avoir des radians
    vFish.setY(-cos(m_angle*DEG2RAD)); //pour avoir des radians

    //--------------------
    alphaGoal = (atan2(vGoal.y(),vGoal.x()) - atan2(vFish.y(),vFish.x()))*RAD2DEG;
    while (fabs(alphaGoal) > 180)
    {
        alphaGoal-= sgn(alphaGoal)*360;
    }

    //! Constant Tangential Velocity PID controller (cf. wheeledRobots.cpp)
    omega = pidController(goalCoord, alphaGoal)*RAD2DEG; //passage en degres

    if (abs(omega)>m_omegaMax)
    {
        omega = sgn(omega)*m_omegaMax;
    }

    return omega;
}

//! this method implements a PID controller on the angular velocity
//! NOTE : this method was retrieved from CATS and readapted to this class
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

//----------------------------------------------------------------------------//
//------------------------------- Setter Methods -----------------------------//
//----------------------------------------------------------------------------//

void FishRobot::setFishRobotID(int fishRobotID)
{
    m_fishRobotID = fishRobotID;
}

void FishRobot::setPosition(QPoint newPosition)
{
    m_position = newPosition;
}

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

void FishRobot::setOmegaMax(int newOmegaMax)
{
    m_omegaMax = newOmegaMax;
}

void FishRobot::setDesiredLinearVel(int newLinearVel)
{
    m_desiredLinearVel = newLinearVel;
}

void FishRobot::setFishRobotDimensions(float newRobotWidth, float newRobotHeight)
{
    m_fishRobotWidth = newRobotWidth;
    m_fishRobotHeight = newRobotHeight;
}

void FishRobot::setPath(std::vector<QPoint> newPath)
{
    m_path.clear();
    m_path = newPath;
}

void FishRobot::setPotentialField(PotentialField* newPotField)
{
    m_potentialField = newPotField;
}

void FishRobot::setDynamicWindow(DynamicWindow* newDynamicWindow)
{
    m_dynamicWindow = newDynamicWindow;
}

//! this method helps determine which path planning method should be used for the robots
void FishRobot::setPathPlanningMethod(PathPlanning newPathPlanning)
{
    m_pathplanning = newPathPlanning;
}

//----------------------------------------------------------------------------//
//------------------------------- Getter Methods -----------------------------//
//----------------------------------------------------------------------------//

QPoint FishRobot::getPosition()
{
    return m_position;
}

float FishRobot::getOrientation()
{
    return m_angle;
}

QPoint FishRobot::getTargetPosition()
{
    return m_target->getPosition();
}

//! this method will get the current linear velocity of the fishRobot.
int FishRobot::getLinearVelocity()
{
    return m_linearVel;
}

//! this method will get the current angular velocity of the fishRobot.
int FishRobot::getAngularVelocity()
{
    return m_omega;
}

//! this method will get the maximum linear velocity for the fishRobots.
int FishRobot::getMaxLinearVelocity()
{
    return m_maxLinearVel;
}

//! this method will get the maximum angular velocity for the fishRobots.
int FishRobot::getMaxAngularVelocity()
{
    return m_omegaMax;
}

int FishRobot::getFishRobotWidth()
{
    return m_fishRobotWidth;
}

int FishRobot::getFishRobotHeight()
{
    return m_fishRobotHeight;
}

//! this method will get the next target given a djikstra path, if there are no
//! other path points it will return the current position.
QPoint FishRobot::getNextPathPoint()
{
    m_path.erase(m_path.begin());

    if (m_path.empty())
    {
        return m_position;
    }

    else
    {
        return m_path.at(0);
    }
}

//! this method returns the path computed by dijkstra's shortest path algorithm
//! for the given fish robot
std::vector<QPoint> FishRobot::getDijkstraPath()
{
    return m_path;
}

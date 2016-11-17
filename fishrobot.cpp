//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 4
//Last Modified :
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#include "fishrobot.h"

FishRobot::FishRobot(Lures *lureptr) : m_angle(315)
{
    //associate to each fish a specific lure
    lure = lureptr;
}

QRectF FishRobot::boundingRect() const
{
    //part of the qGraphicsItem instanciation
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

    /*
    // Tail
    painter->setBrush(QColor(0, 0, 255, 127));
    static QPointF points[3] = {QPointF(0, 5*ratio),
                                QPointF(3*ratio, 10*ratio),
                                QPointF(-3*ratio, 10*ratio)};
    painter->drawPolygon(points,3);

    // Body
    painter->setBrush(QColor(0, 0, 255, 127));
    painter->drawEllipse(-3*ratio, -7*ratio, 7*ratio, 13*ratio);

    // Eyes
    painter->setBrush(scene()->collidingItems(this).isEmpty() ? Qt::white : Qt::darkRed);
    painter->drawEllipse(-3*ratio, -6*ratio, 3*ratio, 3*ratio);
    painter->drawEllipse(1*ratio, -6*ratio, 3*ratio, 3*ratio);
    */
}

void FishRobot::setPath(std::vector<QPoint> newPath)
{
    m_path.clear();
    m_path = newPath;
}

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

void FishRobot::advance(int step = 1)//moves each fish at each step of the program
{
    float   distGoal, alphaGoal, dt = 0.033; // find a way to determine dt
    QPoint  deltaCoord, vGoal, goalCoord;
    QPointF vFish;

    if (!step)
    {
        return;
    }

    if (m_path.empty())
    {
        m_position.setX(650);
        m_position.setY(700);
        m_angle = 315;
        setRotation(m_angle);
        setPos(m_position);
        //goalCoord = lure->getPosition();
        return;
    }
    else
    {
        //find point in path with minimal distance from the fish
        identifyClosestPathPoint();
        goalCoord = m_path.at(0);
    }
    //goalCoord = lure->getPosition();

    //Difference of coordinates between current position and goal
    deltaCoord.setX(goalCoord.x()-m_position.x());
    deltaCoord.setY(goalCoord.y()-m_position.y());

    //Distance to Goal
    distGoal = sqrt(pow((deltaCoord.x()),2) + pow(deltaCoord.y(),2));

    if (!m_path.empty() && distGoal<15)
    {
        m_path.erase(m_path.begin());
        //change the goal
        if (!m_path.empty())
        {
            goalCoord = m_path.at(0);
            //calculate the new distance to the goal
            deltaCoord.setX(goalCoord.x()-m_position.x());
            deltaCoord.setY(goalCoord.y()-m_position.y());
            distGoal = sqrt(pow((deltaCoord.x()),2) + pow(deltaCoord.y(),2));
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

    //angle to Goal (unecessary variables but easier to read)
    vGoal.setX(deltaCoord.x());
    vGoal.setY(deltaCoord.y());
    vFish.setX(sin(m_angle*DEG2RAD));  //pour avoir des radians
    vFish.setY(-cos(m_angle*DEG2RAD)); //pour avoir des radians

    //--------------------
    alphaGoal = (atan2(vGoal.y(),vGoal.x()) - atan2(vFish.y(),vFish.x()))*RAD2DEG;
    if (fabs(alphaGoal) > 180)
    {
        alphaGoal-= sgn(alphaGoal)*360;
    }

    //UNICYCLE MODEL IMPLEMENTATION
    m_vx =  m_linearVel*sin(m_angle*DEG2RAD);
    m_vy = -m_linearVel*cos(m_angle*DEG2RAD);

    //New FishRobot Position
    m_position.setX(m_position.x()+m_vx*dt);
    m_position.setY(m_position.y()+m_vy*dt);

    // Constant Tangential Velocity PID controller (cf. wheeledRobots.cpp)
    m_omega = pidController(goalCoord, alphaGoal)*RAD2DEG; //passage en degres

    if (abs(m_omega)>m_omegaMax)
    {
        m_omega = sgn(m_omega)*m_omegaMax;
    }

    //New FishRobot angle
    m_angle += m_omega*dt;

    //Inverse Kinematics
    m_vl = m_linearVel + m_omega*DIST_WHEELS/2.0; //normalement divisé par le rayon des roues..
    m_vr = m_linearVel - m_omega*DIST_WHEELS/2.0; // idem

    setRotation(m_angle);
    setPos(m_position);
}


//Code récupéré de CATS et adapté pour compiler dans ce fichier.
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

    qDebug()<<"Kp : "<<m_Kp<< "|| Ki : "<<m_Ki<<" || Kd : "<<m_Kd;
    qDebug()<<"omega : "<<AngVel;

    return AngVel;
}

//! Setter Functions

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

void FishRobot::setLinearVel(int newLinearVel)
{
    m_linearVel = newLinearVel;
}

void FishRobot::setFishRobotDimensions(float newRobotWidth, float newRobotHeight)
{
    m_fishRobotWidth = newRobotWidth;
    m_fishRobotHeight = newRobotHeight;
}

//! Getter functions

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
    return lure->getPosition();
}


//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 4
//Last Modified :
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#include "fishrobot.h"

FishRobot::FishRobot(Lures *lureptr) : angle(315)
{
    //associate to each fish a specific lure
    lure = lureptr;
}

QRectF FishRobot::boundingRect() const
{
    //part of the qGraphicsItem instanciation
    qreal adjust = 2;

    return QRectF(-fishRobotWidth/2-adjust,-fishRobotHeight/2-adjust,
                      fishRobotWidth+adjust, fishRobotHeight+adjust);
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
    static QPointF points[3] = {QPointF(0, +fishRobotHeight/4),
                                QPointF(-fishRobotWidth/2, +fishRobotHeight/2),
                                QPointF(+fishRobotWidth/2, +fishRobotHeight/2)};
    painter->drawPolygon(points,3);

    // Body
    painter->setBrush(QColor(0, 0, 255, 127));
    QRectF body(-fishRobotWidth/2, -fishRobotHeight/2, fishRobotWidth, fishRobotHeight*4/5);
    painter->drawEllipse(body);

    // Eyes
    painter->setBrush(scene()->collidingItems(this).isEmpty() ? Qt::white : Qt::darkRed);
    QRectF leftEye(-fishRobotWidth/2, -fishRobotHeight/2, fishRobotWidth/2, fishRobotHeight/4);
    QRectF rightEye(0, -fishRobotHeight/2, fishRobotWidth/2, fishRobotHeight/4);
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
    path = newPath;
}

void FishRobot::identifyClosestPathPoint()
{
    float mindist = INF, distance, index;

    for (auto i = path.begin(); i!= path.end();i++)
    {
        distance = pow((position.x() - i->x()),2) + pow((position.y() - i->y()),2);
        if (distance<mindist)
        {
                mindist = distance;
                index = std::distance(path.begin(),i);
        }
    }

    path.erase(path.begin(), path.begin()+index);
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

    if (path.empty())
    {
        position.setX(450);
        position.setY(500);
        angle = 315;
        setRotation(angle);
        setPos(position);
        return;//goalCoord = lure->getPosition();
    }
    else
    {
        //find point in path with minimal distance from the fish
        identifyClosestPathPoint();
        goalCoord = path.at(0);
    }

    //Difference of coordinates between current position and goal
    deltaCoord.setX(goalCoord.x()-position.x());
    deltaCoord.setY(goalCoord.y()-position.y());

    //Distance to Goal
    distGoal = sqrt(pow((deltaCoord.x()),2) + pow(deltaCoord.y(),2));

    if (!path.empty() && distGoal<10)
    {
        path.erase(path.begin());
        //change the goal
        if (!path.empty())
        {
            goalCoord = path.at(0);
            //calculate the new distance to the goal
            deltaCoord.setX(goalCoord.x()-position.x());
            deltaCoord.setY(goalCoord.y()-position.y());
            distGoal = sqrt(pow((deltaCoord.x()),2) + pow(deltaCoord.y(),2));
        }
        else
        {
            return;
        }
    }
    else if (path.empty() && distGoal<5)
    {
       return;
    }

    //Angle to Goal (unecessary variables but easier to read)
    vGoal.setX(deltaCoord.x());
    vGoal.setY(deltaCoord.y());
    vFish.setX(sin(angle*DEG2RAD));  //pour avoir des radians
    vFish.setY(-cos(angle*DEG2RAD)); //pour avoir des radians

    //--------------------
    alphaGoal = (atan2(vGoal.y(),vGoal.x()) - atan2(vFish.y(),vFish.x()))*RAD2DEG;
    if (fabs(alphaGoal) > 180)
    {
        alphaGoal-= sgn(alphaGoal)*360;
    }

    //UNICYCLE MODEL IMPLEMENTATION
    vx =  linearVel*sin(angle*DEG2RAD);
    vy = -linearVel*cos(angle*DEG2RAD);

    //New FishRobot Position
    position.setX(position.x()+vx*dt);
    position.setY(position.y()+vy*dt);

    // Constant Tangential Velocity PID controller (cf. wheeledRobots.cpp)
    omega = pidController(goalCoord, alphaGoal)*RAD2DEG; //passage en degres

    if (abs(omega)>omegaMax)
    {
        omega = sgn(omega)*omegaMax;
    }

    //New FishRobot Angle
    angle += omega*dt;

    //Inverse Kinematics
    vl = linearVel + omega*DIST_WHEELS/2.0; //normalement divisé par le rayon des roues..
    vr = linearVel - omega*DIST_WHEELS/2.0; // idem

    setRotation(angle);
    setPos(position);
}


void FishRobot::setPosition(QPoint newPosition)
{
    position = newPosition;
}

QPoint FishRobot::getPosition()
{
    return position;
}

//Code récupéré de CATS et adapté pour compiler dans ce fichier.
float FishRobot::pidController(QPoint goalCoord, float alphaGoal)
{
    std::vector<double> tmp(2), PosD(2), Pos(2), T(2);
    static std::vector<double> error_PID(5), error_buffer;
    float rho = 0, alpha = 0, AngVel = 0;

    Pos.at(0) = position.x();
    Pos.at(1) = position.y();
    PosD.at(0) = goalCoord.x();
    PosD.at(1) = goalCoord.y();

    /// PID controller assuming a constant tangential speed
    /// position error calculation
    tmp.at(0) = PosD.at(0) - Pos.at(0);
    tmp.at(1) = PosD.at(1) - Pos.at(1);

    /// Error represented into the robot's coordinate system

    /// T.at(0,0) = cos(Pos.at(2));   T.at(0,1) = sin(Pos.at(2));
    /// T.at(1,0) =-sin(Pos.at(2));   T.at(1,1) = cos(Pos.at(2));

    T.at(0) = tmp.at(0)*cos(angle*DEG2RAD) + tmp.at(1)*sin(angle*DEG2RAD);
    T.at(1) = -tmp.at(0)*sin(angle*DEG2RAD) + tmp.at(1)*cos(angle*DEG2RAD);

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
        AngVel = Kp * error_PID.at(0) + Ki * error_PID.at(2) - Kd * error_PID.at(3);
    }
    AngVel = Kp * error_PID.at(0) + Ki * error_PID.at(2) - Kd * error_PID.at(3);
    if (abs(AngVel)>omegaMax)
    {
        AngVel = sgn(AngVel)*omegaMax;
    }

    qDebug()<<"Kp : "<<Kp<< "|| Ki : "<<Ki<<" || Kd : "<<Kd;
    qDebug()<<"omega : "<<AngVel;

    return AngVel;
}

void FishRobot::setControllerParameters(int gain, double newK)
{
    switch(gain){
        case PROP :
                Kp = newK;
            break;
        case INTEG :
                Ki = newK;
            break;
        case DERIV :
                Kd = newK;
            break;
    }
}

void FishRobot::setOmegaMax(int newOmegaMax)
{
    omegaMax = newOmegaMax;
}

void FishRobot::setLinearVel(int newLinearVel)
{
    linearVel = newLinearVel;
}

void FishRobot::setFishRobotDimensions(float newRobotWidth, float newRobotHeight)
{
    fishRobotWidth = newRobotWidth;
    fishRobotHeight = newRobotHeight;
}

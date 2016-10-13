//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 4
//Last Modified :
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#include "fishrobot.h"

FishRobot::FishRobot(Lures *lureptr) : angle(200)
{
    lure = lureptr;
}

QRectF FishRobot::boundingRect() const
{
    qreal adjust = 2;

    return QRectF(-fishRobotWidth/2-adjust,-fishRobotHeight/2-adjust,
                      fishRobotWidth+adjust, fishRobotHeight+adjust);
}

QPainterPath FishRobot::shape() const
{
    QPainterPath path;
    path.addRect(-10, -20, 20, 40);
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

void FishRobot::advance(int step = 1)//moves each fish at each step of the program
{
    float goalCoord[2], distGoal, alphaGoal;
    float deltaCoord[2], dt = 0.033; // find a way to determine dt
    float vGoal[2], vFish[2];

    if (!step)
    {
        return;
    }

    lure->getPosition(goalCoord);

    //Difference of coordinates between current position and goal
    deltaCoord[0] = goalCoord[0]-position[0];
    deltaCoord[1] = goalCoord[1]-position[1];

    //Distance to Goal
    distGoal = sqrt(pow((deltaCoord[0]),2) + pow(deltaCoord[1],2));

    //if (distGoal<10)
    //    return;

    //Angle to Goal (unecessary variables but easier to read)
    vGoal[0] = deltaCoord[0];
    vGoal[1] = deltaCoord[1];
    vFish[0] =  sin(angle*DEG2RAD); //pour avoir des radians
    vFish[1] = -cos(angle*DEG2RAD);//pour avoir des radians

    //--------------------
    alphaGoal = (atan2(vGoal[1],vGoal[0]) - atan2(vFish[1],vFish[0]))*180/PI;
    if (fabs(alphaGoal)> 180)
    {
        alphaGoal-= sgn(alphaGoal)*360;
    }

    //UNICYCLE MODEL IMPLEMENTATION
    vx =  linearVel*sin(angle*DEG2RAD);
    vy = -linearVel*cos(angle*DEG2RAD);

    //New FishRobot Position

    position[0] += vx*dt;
    position[1] += vy*dt;

    // Constant Tangential Velocity PID controller (cf. wheeledRobots.cpp)

    omega = pidController(goalCoord, alphaGoal)*RAD2DEG; //passage en degres

    if (abs(omega)>omegaMax){
        omega = sgn(omega)*omegaMax;
    }

    /*if(abs(alphaGoal)<1)
    {
        angle=90;
    }
    else*/

    //New FishRobot Angle
    angle += omega*dt;

    //qDebug()<<"omega : "<<omega;

    //Inverse Kinematics
    vl = linearVel + omega*L/2.0; //normalement divisé par le rayon des roues..
    vr = linearVel - omega*L/2.0; // idem

    setRotation(angle);
    setPos(position[0],position[1]);
}


void FishRobot::setPosition(float newPosition[2])
{
    position[0] = newPosition[0];
    position[1] = newPosition[1];
}

void FishRobot::getPosition(float currentPosition[2])
{
    currentPosition[0] = position[0];
    currentPosition[1] = position[1];
}


//Code récupéré de CATS et adapté pour compiler dans ce fichier.
float FishRobot::pidController(float goalCoord[2], float alphaGoal)
{
    std::vector<double> tmp(2), PosD(2), Pos(2), T(2);
    static std::vector<double> error_PID(5), error_buffer;
    float rho = 0, alpha = 0, AngVel = 0;

    Pos.at(0) = position[0];
    Pos.at(1) = position[1];
    PosD.at(0) = goalCoord[0];
    PosD.at(1) = goalCoord[1];

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

    //qDebug()<<"Kp : "<<1000*Kp<< "|| Ki : "<<Ki<<" || Kd : "<<Kd;

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

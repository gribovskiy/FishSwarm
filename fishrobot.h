//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 4
//Last Modified :
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#ifndef FISHROBOT_H
#define FISHROBOT_H

#include <qgraphicsitem.h>
#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>

#include <vector>
#include <QDebug>
#include <math.h>

#include "constants.h"
#include "lures.h"
#include "djikstra.h"

#define sgn(x) ( x != 0 ? abs(x) / x : 0 ) //Pris du Code de WheeledRobot.cpp

#define OMEGA_MAX 360   // 360 degrés/s
#define VLINEAR   30    // 100 pixels/s
#define L         20    //L : distance between the wheels in cm


class FishRobot : public QGraphicsItem
{
public:
    FishRobot(Lures *lurePtr);
    Lures *lure;

    QRectF boundingRect() const Q_DECL_OVERRIDE; //returns the estimated area for the fish drawing
    QPainterPath shape() const Q_DECL_OVERRIDE; //returns the shape of our fish
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) Q_DECL_OVERRIDE;


     void        setPosition(int newPosition[2]); //remove fish
     void        getPosition(int currentPosition[2]); //remove fish - see if already implemented
     static void setControllerParameters(int gain, double newK);
     static void setOmegaMax(int newOmegaMax);
     static void setLinearVel(int newLinearVel);
     static void setFishRobotDimensions(float newRobotWidth, float newRobotHeight);

   protected slots:
     void     advance(int step) Q_DECL_OVERRIDE; //handles the animation
     float    pidController(int goalCoord[2], float alphaGoal);

   private:     
     float    angle, position[2], omega = 0;
     float    prevGoalCoordX = -1, prevGoalCoordY = -1;
     float    vl = 0, vr = 0, vx = 0, vy = 0; //mettre des floats?
};

//INPUTS FROM SIMULATOR

//KPC : entre 1.055 et 1.062
static float    omegaMax = 10, linearVel = 10;
static double   Kp = 1.057, Ki = 0, Kd = 0;
static int      fishRobotWidth = 2, fishRobotHeight = 10; //en pixels
static int      distNodes = 9;

#endif // FISHROBOT_H

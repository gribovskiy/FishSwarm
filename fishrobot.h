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
#define DIST_WHEELS  20 //distance between the wheels in cm


class FishRobot : public QGraphicsItem
{
public:
    FishRobot(Lures *lurePtr);
    Lures *lure;

    QRectF boundingRect() const Q_DECL_OVERRIDE; //returns the estimated area for the fish drawing
    QPainterPath shape() const Q_DECL_OVERRIDE; //returns the shape of our fish
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) Q_DECL_OVERRIDE;

     void        setPath(std::vector<QPoint> newPath);
     void        setPosition(QPoint newPosition); //remove fish
     QPoint      getPosition(); //remove fish - see if already implemented
     static void setControllerParameters(int gain, double newK);
     static void setOmegaMax(int newOmegaMax);
     static void setLinearVel(int newLinearVel);
     static void setFishRobotDimensions(float newRobotWidth, float newRobotHeight);

   protected slots:
     void     advance(int step) Q_DECL_OVERRIDE; //handles the animation
     float    pidController(QPoint goalCoord, float alphaGoal);

   private:
     void     identifyClosestPathPoint();

     float    angle, omega = 0;
     QPoint   position;
     std::vector<QPoint> path;
     float    vl = 0, vr = 0, vx = 0, vy = 0; //mettre des floats?
};

//INPUTS FROM SIMULATOR

//KPC : entre 1.055 et 1.062
static float    omegaMax = 200, linearVel = 10;
static double   Kp = 0.3 /*1.057*/, Ki = 0, Kd = 0;
static int      fishRobotWidth = 2, fishRobotHeight = 10; //en pixels

#endif // FISHROBOT_H

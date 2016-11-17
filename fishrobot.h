//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 4
//Last Modified :
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

// FIXME : need a comment for every method (and for every class), for instance:

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

#define OMEGA_MAX 200   // 360 degrés/s
#define VLINEAR   7    // 100 pixels/s
#define DIST_WHEELS  20 //distance between the wheels in cm


class FishRobot : public QGraphicsItem
{
public:
    FishRobot(Lures *lurePtr);
    Lures *lure;

    QRectF boundingRect() const Q_DECL_OVERRIDE; //returns the estimated area for the fish drawing
    QPainterPath shape() const Q_DECL_OVERRIDE;  //returns the shape of our fish
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) Q_DECL_OVERRIDE;

     void        setPath(std::vector<QPoint> newPath);
     void        setPosition(QPoint newPosition); //remove fish
     QPoint      getPosition(); //remove fish - see if already implemented
     QPoint      getTargetPosition();
     float       getOrientation();
     static void setControllerParameters(Gains gain, double newK);
     static void setOmegaMax(int newOmegaMax);
     static void setLinearVel(int newLinearVel);
     static void setFishRobotDimensions(float newRobotWidth, float newRobotHeight);

   protected slots:
     void     advance(int step) Q_DECL_OVERRIDE; //handles the animation
     float    pidController(QPoint goalCoord, float alphaGoal);

   private:
     void     identifyClosestPathPoint();

     float    m_angle, m_omega = 0;
     QPoint   m_position;
     std::vector<QPoint> m_path;
     float    m_vl = 0, m_vr = 0, m_vx = 0, m_vy = 0; //mettre des floats?
     //TODO: input width and height of the simulator
};

//TODO : INPUTS FROM SIMULATOR

//KPC : entre 1.055 et 1.062
static float    m_omegaMax = 200, m_linearVel = 10;
static double   m_Kp = 0.3 /*1.057*/, m_Ki = 0, m_Kd = 0;
static int      m_fishRobotWidth = 2, m_fishRobotHeight = 10; //en pixels
static int      simulationWidth = 550, simulationHeight = 550;

#endif // FISHROBOT_H

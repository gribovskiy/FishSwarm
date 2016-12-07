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
#include "potentialfield.h"
#include "dynamicwindow.h"

#define OMEGA_MAX 200   // 360 degrés/s
#define VLINEAR   82    // 10 cm/s
#define DIST_WHEELS  20 //distance between the wheels in cm

class PotentialField;
class DynamicWindow;

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
     void        setFishRobotID(int fishRobotID);
     QPoint      getPosition(); //remove fish - see if already implemented
     QPoint      getTargetPosition();
     float       getOrientation();
     int         getLinearVelocity();
     int         getMaxLinearVelocity();
     int         getFishRobotWidth();
     int         getFishRobotHeight();
     int         getAngularVelocity();
     int         getMaxAngularVelocity();

     static void setControllerParameters(Gains gain, double newK);
     static void setOmegaMax(int newOmegaMax);
     static void setDesiredLinearVel(int newLinearVel);
     static void setFishRobotDimensions(float newRobotWidth, float newRobotHeight);
     //!this method helps determine which path planning method should be used for the robots
     static void setPathPlanningMethod(PathPlanning newPathPlanning);
     //! this method gives the fishRobot access to the potential field in order
     //! to determine the velocities at the next step
     static void setPotentialField(PotentialField* newPotField);
     //! this method gives the fishRobot access to the dynamic window in order
     //! to modify trajectory if an obstacle is detected on the current path
     static void setDynamicWindow(DynamicWindow* newDynamicWindow);

   protected slots:
     void     advance(int step) Q_DECL_OVERRIDE; //handles the animation
     float    pidController(QPoint goalCoord, float alphaGoal);


     //this method computes the new linear and angular velocities for the robot given the goal coordinates
     void computeNewVelocitiesAndNewPosition(QPoint goalCoord);
     //this method computes the new angular velocity for the robot given the goal coordinates
     float computeAngularVelocity(QPoint goalCoord);
     //this method gives the following position for the robot using simple PID controller to follow the target
     void advancePID();
     //this method gives the following position for the robot using Djikstra to follow the target
     void advanceDjikstra();
     //this method gives the following position for the robot using Potential Field to follow the target
     void advancePotField();

   private:
     void                  identifyClosestPathPoint();
     float                  m_angle, m_omega = 0;
     int                    m_linearVel;
     QPoint                 m_position;
     std::vector<QPoint>    m_path;
     float                  m_vl = 0, m_vr = 0, m_vx = 0, m_vy = 0; //mettre des floats?
     int                    m_fishRobotID;
     //TODO: input width and height of the simulator

};

static PathPlanning    m_pathplanning = PathPlanning::PID;
static PotentialField *m_potentialField = NULL;
static DynamicWindow  *m_dynamicWindow = NULL;
static float           m_omegaMax = 200, m_desiredLinearVel = 82; //! 16cm/s
static int             m_maxLinearVel = 131; //! 16cm/s
static double          m_Kp = 0.3 /*1.057*/, m_Ki = 0, m_Kd = 0;
static int             m_fishRobotWidth = 2, m_fishRobotHeight = 10; //en pixels


//TODO : INPUTS FROM SIMULATOR

//KPC : entre 1.055 et 1.062
static int             simulationWidth = 750, simulationHeight = 750;


#endif // FISHROBOT_H

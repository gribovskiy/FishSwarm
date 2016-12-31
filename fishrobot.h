//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 4
//Last Modified : 26/12/2016
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
#include "target.h"
#include "potentialfield.h"
#include "dynamicwindow.h"

#define OMEGA_MAX   200 // 360 degrés/s
#define VLINEAR     82  // 10 cm/s
#define DIST_WHEELS 16  //distance between the wheels 2cm

class PotentialField;
class DynamicWindow;

/*!
 * Fish Robot class. This class generates the new position of the fishrobots
 * at each time step using various path planning and obstacle avoidance methods
 */
class FishRobot : public QGraphicsItem
{
public:
    //-------------------------------------------//
    //-----------Class  Constructor--------------//
    //-------------------------------------------//

    /*!
     * Class constructor. Instanciates the fishRobot with its target and ID
     * and sets the intial orientation
     */
    FishRobot(Target *targetPtr, int fishRobotID);

    //-------------------------------------------//
    //-----------QGraphics Functions-------------//
    //-------------------------------------------//

    /*!
     * QGraphicsItem method, it returns the estimated area for the Target drawing
     */

    QRectF boundingRect() const Q_DECL_OVERRIDE; //returns the estimated area for the fish drawing

    /*!
     * QGraphicsItem method, it returns returns the shape of the fishRobot
     */
    QPainterPath shape() const Q_DECL_OVERRIDE;

    /*!
     * QGraphicsItem method, paints the fishRobot
     */
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) Q_DECL_OVERRIDE;


    //-------------------------------------------//
    //-------------Setter Functions--------------//
    //-------------------------------------------//

    /*!
     * Exported Member. This method sets the new dijkstra path.
     */
     void        setDijkstraPath(std::vector<QPoint> newPath);
     /*!
      * Exported Member. This method sets the new position for the fishRobot
      */
     void        setPosition(QPoint newPosition);
     /*!
      * Exported Member. This method sets the new ID for the fishRobot
      */
     void        setFishRobotID(int fishRobotID);
     /*!
      * Exported Member. This method sets the new PID controller paramters
      * It receives as input the type of gain as well as the new value.
      */
     static void setControllerParameters(Gains gain, double newK);
     /*!
      * Exported Member. This method sets the maximum angular velocity
      */
     static void setOmegaMax(int newOmegaMax);
     /*!
      * Exported Member. This method sets the new desired linear velocity
      */
     static void setDesiredLinearVel(int newLinearVel);
     /*!
      * Exported Member. This method sets the new fish robot dimensions (width
      * and height)
      */
     static void setFishRobotDimensions(float newRobotWidth, float newRobotHeight);
     /*!
      * Exported Member. this method helps determine which path planning method
      * should be used for the robots
      */
     static void setPathPlanningMethod(PathPlanning newPathPlanning);
     /*!
      * Exported Member.this method gives the fishRobot access to the potential
      * field in order to determine the velocities at the next step
      */

     static void setPotentialField(PotentialField* newPotField);
     /*!
      * Exported Member. this method gives the fishRobot access to the
      * dynamic window in order to modify trajectory if an obstacle is
      * detected on the current path
      */
     static void setDynamicWindow(DynamicWindow* newDynamicWindow);

     /*!
      * Exported Member. this method sets the admissible distances to the
      * intermediate and final target.
      */
     static void setAdmissibleTargetDistances(int intermediateTargetDist, int finalTargetDist);

     /*!
      * Exported Member. this method stores the chosen dijkstra path type in the
      * simulator. Can be modified by directly incorporation dijkstra to the
      * fishrobot as was done for potential field and dynamic window.
      */
     static void setDijkstraPathType(DijkstraPath pathType);

     //-------------------------------------------//
     //-------------Getter Functions--------------//
     //-------------------------------------------//

     /*!
      * Exported Member. This method gets the fishRobot's position
      */
     QPoint getPosition();

     /*!
      * Exported Member. This method gets the final target's position unless the
      * path planning method is the dijkstra + DWA in which case it gives the
      * position of the next target
      */
     QPoint getTargetPosition();

     /*!
      * Exported Member. This methods returns the orientation of the fish robot
      */
     float  getOrientationDeg();

     /*!
      * Exported Member.this method returns the current linear velocity of the
      * fishRobot.
      */
     int    getLinearVelocity();
     /*!
      * Exported Member.this method returns the maximum linear velocity for the
      * fishRobot.
      */
     int    getMaxLinearVelocity();
     /*!
      * Exported Member. This method returns the fishRobot's width
      */
     int    getFishRobotWidth();
     /*!
      * Exported Member. This method returns the fishRobot's height
      */
     int    getFishRobotHeight();
     /*!
      * Exported Member.this method returns the current angular velocity of the
      * fishRobot.
      */
     int    getAngularVelocity();
     /*!
      * Exported Member. this method returns the maximum angular velocity for
      * the fishRobot.
      */
     int    getMaxAngularVelocity();

     /*!
      * Exported Member.this method will get the next target given a djikstra
      * path, if there are no other path points it will return the current
      * position.
      */
     QPoint getNextPathPoint();

     /*!
      * Exported Member.this method returns the path computed by dijkstra's
      * shortest path algorithm for the given fish robot
      */
     std::vector<QPoint> getDijkstraPath();


   protected slots:
     /*!
      * Exported Member. This QGraphics method handles the animation and computes
      * the movement of each fishRobot at each timestep using the different
      * path planning and obstacle avoidance methods
      */
     void     advance(int step) Q_DECL_OVERRIDE;

   private:

     //! the pointer to the target of the given fishRobot
     Target                *m_target;
     //! the status of the fishRobot
     FishBotStatus          m_status = FishBotStatus::MOVING;
     //! the orientation and rotational velocities
     float                  m_angle, m_omega = 0;
     //! the linear velocity
     int                    m_linearVel;
     //! the current position
     QPoint                 m_position;
     //! the dijkstra path
     std::vector<QPoint>    m_dijkstraPath;
     //! the x, y, left and right wheel velocities
     float                  m_vl = 0, m_vr = 0, m_vx = 0, m_vy = 0;
     int                    m_fishRobotID;
     //! TODO: input width and height of the simulator

     //-------------------------------------------//
     //------------Non Exported Members-----------//
     //-------------------------------------------//

    /*!
    * Non Exported Member. This method identifies the closest point in the
    * Dijkstra path to avoid backtracking.It deletes all points preceding the
    * identified one This method is adapted to the case where all the dijkstra
    * path points are available, not just the reduced path
    */
     void   eliminateBackwardsDijkstraPathPoints();

     /*!
      * Non Exported Member. This method identifies the closest point in the
      * Dijkstra path to avoid backtracking.
      */
     QPoint identifyDjikstraTarget();

     /*!
      * Non Exported Member. This method implements a PID controller on the
      * angular velocity
      * //!NOTE : this method was retrieved from CATS and readapted to this class
      */
     float  pidController(QPoint goalCoord, float alphaGoal);

     /*!
      * Non Exported Member.this method computes the new linear and angular
      * velocities for the robot given the goal coordinates
      */
     void   computeNewVelocitiesAndNewPosition(QPoint goalCoord);

     /*!
      * Non Exported Member. this method computes the angular velocity given
      * the goalCoordinates by determining the angle to the goal and calling
      * the PID controller
      */
     float  computeAngularVelocity(QPoint goalCoord);

     /*!
      * Non Exported Member. this method computes the new position and rotation
      * once the linear velocity and angular velocity have been set.
      */
     void   computeNewPositionAndOrientation();


     /*!
      * Non Exported Member.this method computes the distance between 2 points.
      */
     float computeDistance(QPoint pos1, QPoint pos2);

     /*!
      * Non Exported Member.This method positions the fishRobots and the targets,
      * for Demonstration and evaluation purposes only
      */
     void   placeFishRobotsAndTargets();



     //-------------------------------//
     //---------Advance methods-------//
     //-------------------------------//
     /*!
      * Non Exported Member.this method gives the following position for the
      * robot using simple PID controller to follow the target
      */
     void   advancePID();

     /*!
      * Non Exported Member. this method gives the following position for the
      * robot using Djikstra to follow the target
      */
     void   advanceDjikstra();

     /*!
      * Non Exported Member. this method gives the following position for the
      * robot using Potential Field to follow the target
      */
     void   advancePotField(QPoint goalCoord, int distTarget);

     /*!
      * Non Exported Member. this method gives the following position for the
      * robot using Djikstra with DWA as obstacle avoidace to follow the target
      */
     void  advanceDjikstraDWA();

     /*!
      * Non Exported Member. this method gives the following position for the
      * robot using Djikstra and potential field to follow the target
      */
     void  advanceDjikstraPotField();
};

//! the path planning method chosen
static PathPlanning    m_pathplanning = PathPlanning::PID;
//! the potential field
static PotentialField *m_potentialField = NULL;
//! the dynamic window
static DynamicWindow  *m_dynamicWindow  = NULL;
//! the dijkstra path type
static DijkstraPath    m_dijkstraPathType = DijkstraPath::REDUCED;
//! the maximum angular velocity
static float           m_omegaMax = 200;
//! the desired and maximum linear velocity
static int             m_desiredLinearVel = 82, m_maxLinearVel = 131; //! 16cm/s
//! the gains for the PID controller
//! NOTE : KPC : between 1.055 and 1.062
static double          m_Kp = 0.3 /*1.057*/, m_Ki = 0, m_Kd = 0;
//! the fishRobot dimensions
static int             m_fishRobotWidth = 2, m_fishRobotHeight = 10; //en pixels
//TODO : INPUTS FROM SIMULATOR
static int             m_simulationWidth = 750, m_simulationHeight = 750;
//! the admissible distance to the target
static int             m_targetDist = 5, m_intermediateTargetDist = 20; // in pixels



#endif // FISHROBOT_H

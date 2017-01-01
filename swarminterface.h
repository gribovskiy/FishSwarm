#ifndef SWARMINTERFACE_H
#define SWARMINTERFACE_H

#include <QtWidgets>
#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>
#include <QPixmap>
#include <QDebug>
#include <QtGlobal>
#include <QColor>

#include <math.h>
#include <algorithm>
#include <vector>
#include <iostream>


#include <chrono>
#include <fstream>

#include "fishrobot.h"
#include "target.h"
#include "constants.h"
#include "djikstraboost.h"
#include "potentialfield.h"
#include "dynamicwindow.h"

// FIXME : need a comment for every method (and for every class), for instance:

namespace Ui {
    class SwarmInterface;
}

class SwarmInterface : public QMainWindow
{
    Q_OBJECT

public:
    explicit SwarmInterface(QWidget *parent = 0);
    ~SwarmInterface();


private slots:

    //! This method starts the simulation
    void startSimulation();
    //! This method stops the simulation
    void stopSimulation();
    //! This method prints out the log for the experiments
    void printExperimentsLog();
    //! this method handles the mouse press event to position the targets
    void mousePressEvent(QMouseEvent * mouseEvent);

    //-------------------------------------------//
    //---------------Control Slots---------------//
    //-------------------------------------------//

    //--------------------------//
    //---Simulation Controls----//
    //--------------------------//
    //! this method open's a window to choose the arena
    void on_LoadButton_clicked();
    void on_StartButton_clicked();
    void on_PauseButton_clicked();
    void on_QuitButton_clicked();

    //--------------------------//
    //---Fish Robot Controls----//
    //--------------------------//
    void on_FishSpinBox_valueChanged(int newFishCount);
    void on_LinearVelocityDoubleSpinBox_valueChanged(double newLinearVel);
    void on_OmegaMaxDoubleSpinBox_valueChanged(double newOmegaMax);
    void on_RobotHeightDoubleSpinBox_valueChanged(double newRobotHeight);
    void on_RobotLengthDoubleSpinBox_valueChanged(double newRobotLength);

    //--------------------------//
    //-------PID Controls-------//
    //--------------------------//
    void on_KpDoubleSpinBox_valueChanged(double newKp);
    void on_KiDoubleSpinBox_valueChanged(double newKi);
    void on_KdDoubleSpinBox_valueChanged(double newKd);
    void on_FinalTargetDistance_valueChanged(double arg1);

    //--------------------------//
    //------Arena Controls------//
    //--------------------------//
    void on_ArenaHeightDoubleSpinBox_valueChanged(double newArenaHeight);
    void on_ArenaLengthDoubleSpinBox_valueChanged(double newArenaLength);

    //--------------------------//
    //--Path Planning Controls--//
    //--------------------------//
    void on_PathPlanningComboBox_currentIndexChanged(int index);

    //--------------------------//
    //----Dijkstra Controls-----//
    //--------------------------//
    void on_DJikstraDrawPath_clicked();
    void on_DijkstraIntermediateTargetDistance_valueChanged(double arg1);
    void on_DijkstraPathComboBox_currentIndexChanged(int index);

    //--------------------------//
    //-Potential Field Controls-//
    //--------------------------//
    void on_attractiveDist_valueChanged(double arg1);
    void on_attractiveForce_valueChanged(double arg1);
    void on_InfluenceAngle_valueChanged(double arg1);
    void on_MaxForce_valueChanged(double arg1);
    void on_RobotRepulsiveDist_valueChanged(double arg1);
    void on_RobotsRepulsiveForce_valueChanged(double arg1);
    void on_ArenaRepulsiveDist_valueChanged(double arg1);
    void on_ArenaRepulsiveForce_valueChanged(double arg1);

    //--------------------------//
    //-Dynamic Window Controls--//
    //--------------------------//
    void on_alpha_spinbox_valueChanged(int arg1);
    void on_beta_spinbox_valueChanged(int arg1);
    void on_gamma_spinbox_valueChanged(int arg1);
    void on_delta_spinbox_valueChanged(int arg1);
    void on_DWAdistGoalLimit_valueChanged(double arg1);
    void on_DWAAngleLimit_valueChanged(const QString &arg1);
    void on_DWARobotDistLimit_valueChanged(double arg1);

    void on_UpdateExp_clicked();

private:
    Ui::SwarmInterface   *ui;
    QGraphicsScene       *scene;

    QTimer               m_timer;
    QPixmap              m_imagePixmap;
    QImage               m_imageObject;

    std::ofstream        m_myfile;

    std::vector<FishRobot*>         m_fishRobots;
    std::vector<Target*>             m_targets;
    std::vector<std::vector<enum State>> m_configurationSpace;

    PathPlanning m_pathplanning    = PathPlanning::PID;

    //! Simulator Objects
    float        m_scaleFactor;
    int          m_fishRobotsCount = 0;
    bool         m_simulationOn    = true;

    //-------------------------------------------//
    //----------Djikstra Related Obejcts---------//
    //-------------------------------------------//

    //! djikstra configuration space
    DjikstraBoost*     m_djikstraFishRobots;
    //! djikstra goals
    std::vector<QPoint>                m_goalFishRobots;
    //! the new goals
    std::vector<QGraphicsEllipseItem*> m_pointPlacedFishRobots;
    //! the path items
    std::vector<std::vector<QGraphicsEllipseItem*>> m_djikstraFishRobotsPoints;
    //! the path coordinates
    std::vector<std::vector<QPoint>>                m_djikstraFishRobotsPath;

    //-------------------------------------------//
    //-----Potential Field Related Obejcts-------//
    //-------------------------------------------//
    PotentialField* m_potentialField;

    //-------------------------------------------//
    //-----Dyanmic Window Related Obejcts-------//
    //-------------------------------------------//
    DynamicWindow* m_dynamicWindow;

    //-------------------------------------------//
    //------Methods Related to the Simulation----//
    //-------------------------------------------//

    //! this method instanciates the fish robots by calling their constructor
    //! intiializing their targets and positioning them in the simulation
    void initializeFishRobots();
    //! this method sets up the new scene and new configuration space
    void initializeScene();
    //! this method clears the scene of all its objects
    void clearScene();
    //! this method calculates the new dimensions the fish robots should have in the simulation
    void scaleFishRobots();
    //! this method positions all the fish robots in the simulation
    void positionFishRobots(int newFishCount);
    //! this method deletes all the objects in the simulation
    void deleteAllObjects();
    //! this method calculates the new scale factor when the arena dimensions are changed
    void newScaleFactor();

    //-------------------------------------------//
    //-----Path Planning Methods - Djikstra------//
    //-------------------------------------------//

    //! this method resets the Djikstra grid
    void resizeDijkstra();
    //! this method sets the goal for the given fishRobot
    void djikstraSetGoal(int index);
    //! this method draws the path in the simulation for the chosen fish robots
    void drawDjikstraFishRobot(int index);
    //! this method instanciates Djikstra by calling the constructor
    void initializeDijkstra();

    //-------------------------------------------//
    //-----Path Planning Methods - Pot Field-----//
    //-------------------------------------------//

    //! this method instanciates the potential field by calling the constructor
    void initializePotentialField();
    //! this method updates all the potential field parameters
    void updatePotentialFieldParameters();

    //-------------------------------------------//
    //-----Path Planning Methods - DWA-----------//
    //-------------------------------------------//

    //! this method instanciates the dynamix window by calling the constructor
    void initializeDynamicWindow();
    //! this method updates all the DWA parameters
    void updateDWAParameters();
};




#endif // SwarmInterface_H

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
    // FIXME other methods : on_LinearVelocitySpinBox_valueChanged -> onLinearVelocityValueChanged.
    // Don't forget to change them in the QtDesigner as well.

    void startSimulation();
    void stopSimulation();
    void mousePressEvent(QMouseEvent * mouseEvent);


    //-------------------------------------------//
    //---------------Control Slots---------------//
    //-------------------------------------------//

    //! simulation controls
    void on_LoadButton_clicked();
    void on_StartButton_clicked();
    void on_PauseButton_clicked();

    //! FishRobot controls
    void on_FishSpinBox_valueChanged(int newFishCount);
    void on_LinearVelocitySpinBox_valueChanged(int newLinearVel);
    void on_OmegaMaxSpinBox_valueChanged(int newOmegaMax);
    void on_RobotHeightSpinBox_valueChanged(int newRobotHeight);
    void on_RobotLengthSpinBox_valueChanged(int newRobotLength);

    //! PID controls
    void on_KpSpinBox_valueChanged(int newKp);
    void on_KiSpinBox_valueChanged(int newKi);
    void on_KdSpinBox_valueChanged(int newKd);

    //! Arena Controls
    void on_ArenaHeightSpinBox_valueChanged(int newArenaHeight);
    void on_ArenaLengthSpinBox_valueChanged(int newArenaLength);

    //! Path Planning Controls
    void on_PathPlanningComboBox_currentIndexChanged(int index);

    //! Djikstra Controls
    void on_DJikstraDrawPath_clicked();

    //! Potential Field Controls
    void on_attractiveDist_valueChanged(int arg1);
    void on_attractiveForce_valueChanged(int arg1);
    void on_InfluenceAngle_valueChanged(int arg1);
    void on_MaxForce_valueChanged(int arg1);
    void on_RobotRepulsiveDist_valueChanged(int arg1);
    void on_RobotsRepulsiveForce_valueChanged(int arg1);
    void on_ArenaRepulsiveDist_valueChanged(int arg1);
    void on_ArenaRepulsiveForce_valueChanged(int arg1);

    void on_alpha_spinbox_valueChanged(int arg1);

    void on_beta_spinbox_valueChanged(int arg1);

    void on_gamma_spinbox_valueChanged(int arg1);

    void on_delta_spinbox_valueChanged(int arg1);

private:
    Ui::SwarmInterface   *ui;
    QGraphicsScene       *scene;

    QTimer               m_timer;
    QPixmap              m_imagePixmap;
    QImage               m_imageObject;

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
    void resizeDjikstra();
    //! this method sets the goal for the given fishRobot
    void djikstraSetGoal(int index);
    //! this method draws the path in the simulation for the chosen fish robots
    void drawDjikstraFishRobot(int index);
    //! this method instanciates Djikstra by calling the constructor
    void initializeDjikstra();

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

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
#include "lures.h"
#include "constants.h"
#include "djikstraboost.h"

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
    // FIXME : use the doxygen format for comments //!
    // Don't forget to change them in the QtDesigner as well.

    void startSimulation();
    void stopSimulation();
    void on_LoadButton_clicked();
    void on_StartButton_clicked();
    void on_PauseButton_clicked();
    void on_FishSpinBox_valueChanged(int newFishCount);
    void on_KpSpinBox_valueChanged(int newKp);
    void on_KiSpinBox_valueChanged(int newKi);
    void on_KdSpinBox_valueChanged(int newKd);
    void on_LinearVelocitySpinBox_valueChanged(int newLinearVel);
    void on_OmegaMaxSpinBox_valueChanged(int newOmegaMax);
    void on_ArenaHeightSpinBox_valueChanged(int newArenaHeight);
    void on_ArenaLengthSpinBox_valueChanged(int newArenaLength);
    void on_RobotHeightSpinBox_valueChanged(int newRobotHeight);
    void on_RobotLengthSpinBox_valueChanged(int newRobotLength);
    void mousePressEvent(QMouseEvent * mouseEvent);
    void on_DJikstraDrawPath_clicked();
    void on_DjikstraComboBox_currentIndexChanged(int index);

private:
    Ui::SwarmInterface   *ui;
    QGraphicsScene       *scene;

    QTimer               m_timer;
    QPixmap              m_imagePixmap;
    QImage               m_imageObject;


    std::vector<FishRobot*>         m_fishRobots;
    std::vector<Lures*>             m_lures;
    std::vector<std::vector<enum State>> m_configurationSpace;

    //! Djikstra Objects
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

    float m_scaleFactor;

    void initializeFishRobots();
    void initializeScene();
    void clearScene();
    void scaleFishRobots();
    void positionFishRobots(int newFishCount);
    void deleteAllObjects();
    void initializeDjikstra();
    void resizeDjikstra();
    void djikstraSetGoal(int index);
    void drawDjikstraFishRobot(int index);
    void newScaleFactor();
};

static int m_fishRobotsCount = 0;
static bool m_simulationOn = true;

#endif // SwarmInterface_H

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
    void SwarmInterface_StartSimulation();
    void SwarmInterface_StopSimulation();
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
    void on_DJikstraDrawPathFish1_clicked();
    void mousePressEvent(QMouseEvent * mouseEvent);

private:
    Ui::SwarmInterface*ui;
    QGraphicsScene    *scene;
    QPoint             goalFishRobot1;
    QTimer             timer;
    QPixmap            imagePixmap;
    QImage             imageObject;

    DjikstraBoost* djikstraFishRobot1;
    std::vector <FishRobot*>        fishRobots;
    std::vector <Lures*>            lures;
    std::vector< std::vector<int> > configurationSpace;
    QGraphicsEllipseItem              *pointPlacedFishRobot1 = NULL;
    std::vector<QGraphicsEllipseItem*> djikstraFishRobot1Points;

    float scaleFactor;
    void SwarmInterface_InitializeFishRobots();
    void SwarmInterface_InitializeScene();
    void SwarmInterface_ClearScene();
    void SwarmInterface_ScaleFishRobots();
    void SwarmInterface_PositionFishRobots(int newFishCount);
    void SwarmInterface_DeleteAllObjects();
};

static int fishRobotsCount = 0;
static bool simulationOn = true;

#endif // SwarmInterface_H

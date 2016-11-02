/********************************************************************************
** Form generated from reading UI file 'swarminterface.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SWARMINTERFACE_H
#define UI_SWARMINTERFACE_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SwarmInterface
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout_2;
    QGroupBox *InputsGroupBox;
    QSpinBox *ArenaHeightSpinBox;
    QLabel *ArenaLengthLabel;
    QLabel *RobotLengthLabel;
    QLabel *ArenaHeightLabel;
    QLabel *LinearVelocityLabel;
    QSpinBox *LinearVelocitySpinBox;
    QSpinBox *ArenaLengthSpinBox;
    QSpinBox *RobotLengthSpinBox;
    QLabel *RobotHeightLabel;
    QSpinBox *RobotHeightSpinBox;
    QPushButton *LoadButton;
    QPushButton *StartButton;
    QPushButton *PauseButton;
    QPushButton *QuitButton;
    QSpinBox *FishSpinBox;
    QLabel *FishLabel;
    QLabel *UnitsLabel;
    QLabel *OmegaMaxLabel;
    QSpinBox *OmegaMaxSpinBox;
    QSpinBox *KpSpinBox;
    QLabel *KpLabel;
    QSpinBox *KiSpinBox;
    QLabel *KiLabel;
    QSpinBox *KdSpinBox;
    QLabel *KdLabel;
    QLabel *CharacteristicsLabel;
    QPushButton *DJikstraDrawPathFish1;
    QHBoxLayout *horizontalLayout;
    QGraphicsView *SimulationView;
    QMenuBar *menubar;
    QMenu *menuFish_Swarm_Simulation;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *SwarmInterface)
    {
        if (SwarmInterface->objectName().isEmpty())
            SwarmInterface->setObjectName(QStringLiteral("SwarmInterface"));
        SwarmInterface->resize(782, 640);
        centralwidget = new QWidget(SwarmInterface);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        gridLayout_2 = new QGridLayout(centralwidget);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        InputsGroupBox = new QGroupBox(centralwidget);
        InputsGroupBox->setObjectName(QStringLiteral("InputsGroupBox"));
        InputsGroupBox->setMinimumSize(QSize(200, 0));
        InputsGroupBox->setMaximumSize(QSize(600, 600));
        ArenaHeightSpinBox = new QSpinBox(InputsGroupBox);
        ArenaHeightSpinBox->setObjectName(QStringLiteral("ArenaHeightSpinBox"));
        ArenaHeightSpinBox->setGeometry(QRect(120, 270, 80, 25));
        ArenaHeightSpinBox->setMaximumSize(QSize(80, 16777215));
        QFont font;
        font.setFamily(QStringLiteral("Comic Sans MS"));
        ArenaHeightSpinBox->setFont(font);
        ArenaHeightSpinBox->setMinimum(10);
        ArenaHeightSpinBox->setMaximum(1000);
        ArenaHeightSpinBox->setSingleStep(10);
        ArenaHeightSpinBox->setValue(50);
        ArenaLengthLabel = new QLabel(InputsGroupBox);
        ArenaLengthLabel->setObjectName(QStringLiteral("ArenaLengthLabel"));
        ArenaLengthLabel->setGeometry(QRect(20, 300, 85, 24));
        ArenaLengthLabel->setFont(font);
        RobotLengthLabel = new QLabel(InputsGroupBox);
        RobotLengthLabel->setObjectName(QStringLiteral("RobotLengthLabel"));
        RobotLengthLabel->setGeometry(QRect(20, 360, 85, 25));
        RobotLengthLabel->setFont(font);
        ArenaHeightLabel = new QLabel(InputsGroupBox);
        ArenaHeightLabel->setObjectName(QStringLiteral("ArenaHeightLabel"));
        ArenaHeightLabel->setGeometry(QRect(20, 270, 111, 25));
        QFont font1;
        font1.setFamily(QStringLiteral("Comic Sans MS"));
        font1.setPointSize(13);
        ArenaHeightLabel->setFont(font1);
        LinearVelocityLabel = new QLabel(InputsGroupBox);
        LinearVelocityLabel->setObjectName(QStringLiteral("LinearVelocityLabel"));
        LinearVelocityLabel->setGeometry(QRect(20, 90, 101, 25));
        LinearVelocityLabel->setFont(font);
        LinearVelocitySpinBox = new QSpinBox(InputsGroupBox);
        LinearVelocitySpinBox->setObjectName(QStringLiteral("LinearVelocitySpinBox"));
        LinearVelocitySpinBox->setGeometry(QRect(120, 90, 80, 25));
        LinearVelocitySpinBox->setMaximumSize(QSize(80, 16777215));
        LinearVelocitySpinBox->setMinimum(1);
        LinearVelocitySpinBox->setMaximum(1000);
        LinearVelocitySpinBox->setValue(70);
        ArenaLengthSpinBox = new QSpinBox(InputsGroupBox);
        ArenaLengthSpinBox->setObjectName(QStringLiteral("ArenaLengthSpinBox"));
        ArenaLengthSpinBox->setGeometry(QRect(120, 300, 80, 24));
        ArenaLengthSpinBox->setMaximumSize(QSize(80, 16777215));
        ArenaLengthSpinBox->setFont(font);
        ArenaLengthSpinBox->setMinimum(10);
        ArenaLengthSpinBox->setMaximum(1000);
        ArenaLengthSpinBox->setSingleStep(10);
        ArenaLengthSpinBox->setValue(50);
        RobotLengthSpinBox = new QSpinBox(InputsGroupBox);
        RobotLengthSpinBox->setObjectName(QStringLiteral("RobotLengthSpinBox"));
        RobotLengthSpinBox->setGeometry(QRect(120, 360, 80, 25));
        RobotLengthSpinBox->setMaximumSize(QSize(80, 16777215));
        RobotLengthSpinBox->setFont(font);
        RobotLengthSpinBox->setValue(1);
        RobotHeightLabel = new QLabel(InputsGroupBox);
        RobotHeightLabel->setObjectName(QStringLiteral("RobotHeightLabel"));
        RobotHeightLabel->setGeometry(QRect(20, 330, 85, 25));
        RobotHeightLabel->setFont(font);
        RobotHeightSpinBox = new QSpinBox(InputsGroupBox);
        RobotHeightSpinBox->setObjectName(QStringLiteral("RobotHeightSpinBox"));
        RobotHeightSpinBox->setGeometry(QRect(120, 330, 80, 25));
        RobotHeightSpinBox->setMaximumSize(QSize(80, 16777215));
        RobotHeightSpinBox->setFont(font);
        RobotHeightSpinBox->setValue(3);
        RobotHeightSpinBox->setDisplayIntegerBase(10);
        LoadButton = new QPushButton(InputsGroupBox);
        LoadButton->setObjectName(QStringLiteral("LoadButton"));
        LoadButton->setGeometry(QRect(20, 400, 181, 24));
        StartButton = new QPushButton(InputsGroupBox);
        StartButton->setObjectName(QStringLiteral("StartButton"));
        StartButton->setGeometry(QRect(20, 460, 181, 24));
        StartButton->setAutoFillBackground(false);
        PauseButton = new QPushButton(InputsGroupBox);
        PauseButton->setObjectName(QStringLiteral("PauseButton"));
        PauseButton->setGeometry(QRect(20, 490, 181, 24));
        QuitButton = new QPushButton(InputsGroupBox);
        QuitButton->setObjectName(QStringLiteral("QuitButton"));
        QuitButton->setGeometry(QRect(20, 520, 181, 24));
        FishSpinBox = new QSpinBox(InputsGroupBox);
        FishSpinBox->setObjectName(QStringLiteral("FishSpinBox"));
        FishSpinBox->setGeometry(QRect(121, 30, 80, 25));
        FishSpinBox->setMaximumSize(QSize(80, 16777215));
        FishSpinBox->setFont(font);
        FishSpinBox->setMinimum(0);
        FishSpinBox->setValue(1);
        FishLabel = new QLabel(InputsGroupBox);
        FishLabel->setObjectName(QStringLiteral("FishLabel"));
        FishLabel->setGeometry(QRect(20, 29, 85, 25));
        FishLabel->setFont(font1);
        UnitsLabel = new QLabel(InputsGroupBox);
        UnitsLabel->setObjectName(QStringLiteral("UnitsLabel"));
        UnitsLabel->setGeometry(QRect(10, 60, 171, 25));
        QFont font2;
        font2.setFamily(QStringLiteral("Comic Sans MS"));
        font2.setPointSize(14);
        font2.setBold(true);
        font2.setItalic(true);
        font2.setWeight(75);
        UnitsLabel->setFont(font2);
        OmegaMaxLabel = new QLabel(InputsGroupBox);
        OmegaMaxLabel->setObjectName(QStringLiteral("OmegaMaxLabel"));
        OmegaMaxLabel->setGeometry(QRect(20, 120, 101, 25));
        OmegaMaxLabel->setFont(font);
        OmegaMaxSpinBox = new QSpinBox(InputsGroupBox);
        OmegaMaxSpinBox->setObjectName(QStringLiteral("OmegaMaxSpinBox"));
        OmegaMaxSpinBox->setGeometry(QRect(120, 120, 80, 25));
        OmegaMaxSpinBox->setMaximumSize(QSize(80, 16777215));
        OmegaMaxSpinBox->setMinimum(1);
        OmegaMaxSpinBox->setMaximum(1000);
        OmegaMaxSpinBox->setValue(100);
        KpSpinBox = new QSpinBox(InputsGroupBox);
        KpSpinBox->setObjectName(QStringLiteral("KpSpinBox"));
        KpSpinBox->setGeometry(QRect(120, 150, 80, 25));
        KpSpinBox->setMaximumSize(QSize(80, 16777215));
        KpSpinBox->setMinimum(1);
        KpSpinBox->setMaximum(10000);
        KpSpinBox->setSingleStep(1);
        KpSpinBox->setValue(1057);
        KpLabel = new QLabel(InputsGroupBox);
        KpLabel->setObjectName(QStringLiteral("KpLabel"));
        KpLabel->setGeometry(QRect(20, 150, 101, 25));
        KpLabel->setFont(font);
        KiSpinBox = new QSpinBox(InputsGroupBox);
        KiSpinBox->setObjectName(QStringLiteral("KiSpinBox"));
        KiSpinBox->setGeometry(QRect(120, 180, 80, 25));
        KiSpinBox->setMaximumSize(QSize(80, 16777215));
        KiSpinBox->setMinimum(0);
        KiSpinBox->setMaximum(10000);
        KiSpinBox->setSingleStep(1);
        KiSpinBox->setValue(0);
        KiLabel = new QLabel(InputsGroupBox);
        KiLabel->setObjectName(QStringLiteral("KiLabel"));
        KiLabel->setGeometry(QRect(20, 180, 101, 25));
        KiLabel->setFont(font);
        KdSpinBox = new QSpinBox(InputsGroupBox);
        KdSpinBox->setObjectName(QStringLiteral("KdSpinBox"));
        KdSpinBox->setGeometry(QRect(120, 210, 80, 25));
        KdSpinBox->setMaximumSize(QSize(80, 16777215));
        KdSpinBox->setMinimum(0);
        KdSpinBox->setMaximum(10000);
        KdSpinBox->setSingleStep(1);
        KdSpinBox->setValue(0);
        KdLabel = new QLabel(InputsGroupBox);
        KdLabel->setObjectName(QStringLiteral("KdLabel"));
        KdLabel->setGeometry(QRect(20, 210, 101, 25));
        KdLabel->setFont(font);
        CharacteristicsLabel = new QLabel(InputsGroupBox);
        CharacteristicsLabel->setObjectName(QStringLiteral("CharacteristicsLabel"));
        CharacteristicsLabel->setGeometry(QRect(10, 240, 181, 25));
        CharacteristicsLabel->setFont(font2);
        DJikstraDrawPathFish1 = new QPushButton(InputsGroupBox);
        DJikstraDrawPathFish1->setObjectName(QStringLiteral("DJikstraDrawPathFish1"));
        DJikstraDrawPathFish1->setGeometry(QRect(20, 430, 181, 24));

        gridLayout_2->addWidget(InputsGroupBox, 0, 1, 1, 1);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));

        gridLayout_2->addLayout(horizontalLayout, 0, 3, 1, 1);

        SimulationView = new QGraphicsView(centralwidget);
        SimulationView->setObjectName(QStringLiteral("SimulationView"));
        SimulationView->setMinimumSize(QSize(550, 550));
        SimulationView->setMaximumSize(QSize(550, 550));
        SimulationView->setMouseTracking(true);
        SimulationView->setAcceptDrops(false);
        SimulationView->setAutoFillBackground(true);
        SimulationView->setFrameShape(QFrame::StyledPanel);
        SimulationView->setFrameShadow(QFrame::Raised);

        gridLayout_2->addWidget(SimulationView, 0, 0, 1, 1);

        SwarmInterface->setCentralWidget(centralwidget);
        menubar = new QMenuBar(SwarmInterface);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 782, 22));
        menuFish_Swarm_Simulation = new QMenu(menubar);
        menuFish_Swarm_Simulation->setObjectName(QStringLiteral("menuFish_Swarm_Simulation"));
        SwarmInterface->setMenuBar(menubar);
        statusbar = new QStatusBar(SwarmInterface);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        SwarmInterface->setStatusBar(statusbar);
#ifndef QT_NO_SHORTCUT
        FishLabel->setBuddy(FishSpinBox);
#endif // QT_NO_SHORTCUT

        menubar->addAction(menuFish_Swarm_Simulation->menuAction());
        menuFish_Swarm_Simulation->addSeparator();

        retranslateUi(SwarmInterface);
        QObject::connect(QuitButton, SIGNAL(clicked()), SwarmInterface, SLOT(close()));

        QMetaObject::connectSlotsByName(SwarmInterface);
    } // setupUi

    void retranslateUi(QMainWindow *SwarmInterface)
    {
        SwarmInterface->setWindowTitle(QApplication::translate("SwarmInterface", "MainWindow", 0));
        InputsGroupBox->setTitle(QApplication::translate("SwarmInterface", "Simulation Inputs", 0));
        ArenaLengthLabel->setText(QApplication::translate("SwarmInterface", "Arena Width", 0));
        RobotLengthLabel->setText(QApplication::translate("SwarmInterface", "Robot Width", 0));
        ArenaHeightLabel->setText(QApplication::translate("SwarmInterface", "Arena Height ", 0));
        LinearVelocityLabel->setText(QApplication::translate("SwarmInterface", "Linear Velocity", 0));
        RobotHeightLabel->setText(QApplication::translate("SwarmInterface", "Robot Height", 0));
        LoadButton->setText(QApplication::translate("SwarmInterface", "Load Aquarium", 0));
        StartButton->setText(QApplication::translate("SwarmInterface", "Start", 0));
        PauseButton->setText(QApplication::translate("SwarmInterface", "Pause", 0));
        QuitButton->setText(QApplication::translate("SwarmInterface", "Quit", 0));
        FishLabel->setText(QApplication::translate("SwarmInterface", "FishRobots", 0));
        UnitsLabel->setText(QApplication::translate("SwarmInterface", "Units : cm, s, degrees", 0));
        OmegaMaxLabel->setText(QApplication::translate("SwarmInterface", "Omega Max", 0));
        KpLabel->setText(QApplication::translate("SwarmInterface", "1000Kp", 0));
        KiLabel->setText(QApplication::translate("SwarmInterface", "1000Ki", 0));
        KdLabel->setText(QApplication::translate("SwarmInterface", "1000Kd", 0));
        CharacteristicsLabel->setText(QApplication::translate("SwarmInterface", "Experience Characteristics", 0));
        DJikstraDrawPathFish1->setText(QApplication::translate("SwarmInterface", "Djisktra : FishRobot 1", 0));
        menuFish_Swarm_Simulation->setTitle(QApplication::translate("SwarmInterface", "Fish Swarm Simulation", 0));
    } // retranslateUi

};

namespace Ui {
    class SwarmInterface: public Ui_SwarmInterface {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SWARMINTERFACE_H

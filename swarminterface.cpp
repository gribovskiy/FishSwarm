//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 6
//Last Modified : 26/12/2016
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#include "swarminterface.h"
#include "ui_swarminterface.h"

SwarmInterface::SwarmInterface(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SwarmInterface)
{
    ui->setupUi(this);

    //! put 0 as the default for the choice of fish robots for which to compute
    //! Dijkstra's shortest path
    ui->DjikstraComboBox->addItem(QString::number(0));

    //! define the parameters of the view
    ui->SimulationView->setHorizontalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
    ui->SimulationView->setVerticalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );

    //! load and image and set up the scene
    m_imageObject.load(":/Arenes/Images/arena_triang.png");
    scene = new QGraphicsScene();
    newScaleFactor();
    initializeScene(); //initializes djikstra for the scene
    //! set up the fishRobots
    initializeFishRobots();

    //! Initialize the random function
    std::srand(std::time(0));

    //! start the simulation and the timer
    startSimulation();
}

SwarmInterface::~SwarmInterface()
{
    deleteAllObjects();
    delete ui;
}

//-----------------------------------------------------------------//
//----Functions related to the Initialisation of the Simulation----//
//-----------------------------------------------------------------//

//! this method intializes everything for the fish robots : path planning method
//! PID controller parameters, velocities, distances, and positions them in the
//! scene
void SwarmInterface::initializeFishRobots()
{
    //! give the path planning method to use
    m_pathplanning = (PathPlanning)ui->PathPlanningComboBox->currentIndex();

    FishRobot::setPathPlanningMethod(m_pathplanning);

    //! Set the correct dimensions for the fish robots
    scaleFishRobots();

    //! Set up the PID controller parameters
    FishRobot::setControllerParameters(Gains::PROP, ui->KpDoubleSpinBox->value());
    FishRobot::setControllerParameters(Gains::INTEG, ui->KiDoubleSpinBox->value());
    FishRobot::setControllerParameters(Gains::DERIV, ui->KdDoubleSpinBox->value());

    //! Set the desired linear speed and max angular rotation
    FishRobot::setDesiredLinearVel(ui->LinearVelocityDoubleSpinBox->value()*m_scaleFactor);
    FishRobot::setOmegaMax(ui->OmegaMaxDoubleSpinBox->value());

    //! set the admissible distances to the targets
    FishRobot::setAdmissibleTargetDistances(ui->FinalTargetDistance->value()*m_scaleFactor,
                                            ui->DijkstraIntermediateTargetDistance->value()*m_scaleFactor);
    //! set the dijkstra path type
    FishRobot::setDijkstraPathType((DijkstraPath)ui->DijkstraPathComboBox->currentIndex());

    //! Position the fishRobots in the simulation
    positionFishRobots(ui->FishSpinBox->value());
}

//! this method initializes Dijkstra's shortest path algorithm
void SwarmInterface::initializeDijkstra()
{
    float distNodes = (float) ui->DistanceNodes_spinbox->value()*m_scaleFactor;
    m_djikstraFishRobots = new DjikstraBoost(distNodes, m_configurationSpace);
    //Set up the djikstra shortest path algorithm, currently for the first fish
    //distance between the nodes, to be incorporated to the ui
    resizeDijkstra();
}

//! this method adapts the length of the vectors used for dijkstra when the
//! fish robots count is modified
void SwarmInterface::resizeDijkstra()
{
    //! resize the
    m_goalFishRobots.resize(m_fishRobotsCount);
    m_pointPlacedFishRobots.resize(m_fishRobotsCount);
    m_djikstraFishRobotsPoints.resize(m_fishRobotsCount);
    m_djikstraFishRobotsPath.resize(m_fishRobotsCount);
}

//! this method initializes the potnetial field algorithm
void SwarmInterface::initializePotentialField()
{
    m_potentialField = new PotentialField(m_configurationSpace, &m_fishRobots);
    updatePotentialFieldParameters();
    FishRobot::setPotentialField(m_potentialField);
}

//! this method instanciates the dynamix window by calling the constructor
void SwarmInterface::initializeDynamicWindow()
{
    m_dynamicWindow = new DynamicWindow(m_configurationSpace, &m_fishRobots);
    updateDWAParameters();
    FishRobot::setDynamicWindow(m_dynamicWindow);
}

//! this method updates all the DWA parameters
void SwarmInterface::updateDWAParameters()
{
    int newAlpha = ui->alpha_spinbox->value();
    int newBeta = ui->beta_spinbox->value();
    int newGamma = ui->gamma_spinbox->value();
    int newDelta = ui->delta_spinbox->value();
    float newDistLimitRobot = (float)(ui->DWAdistGoalLimit->value())*m_scaleFactor;
    float newAngleLimitRAD = (float)(ui->DWAAngleLimit->value())*DEG2RAD;
    float newOccupiedRadius = (float)(ui->DWAOccupiedRadiusLimit->value())*m_scaleFactor;


    m_dynamicWindow-> setObjectiveFunctionParameters(newAlpha, newBeta, newGamma, newDelta);
    m_dynamicWindow-> setOccupiedZoneParameters(newDistLimitRobot, newAngleLimitRAD, newOccupiedRadius);

    qDebug()<<"Scale Factor"<<m_scaleFactor<<"new occupied radius"<<newOccupiedRadius;
}

//! this method starts the simulation and updates the scene
void SwarmInterface::startSimulation()
{   
    connect(&m_timer, SIGNAL(timeout()), this, SLOT(printExperimentsLog()));
    m_timer.start(1000 / 33); //equivalent to 30 Hz
    scene->update();
}


//! Non Exported Member.This method prints out the log for the experiments,
//! for evaluation purposes only
void SwarmInterface::printExperimentsLog()
{
    /*
    float vel, distance;
    QPoint pos;
    int method = ui->PathPlanningComboBox->currentIndex();
    //! for the first entry
    static auto t1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time;


    if (!m_myfile.is_open())
    {
        qDebug()<<"OPENING";
        m_myfile.open ("TESTBLOOP.txt");
        m_myfile << "Path Planning Method \t Time Step\t "
                    "Robot1X \t Robot1Y \t Robot1V \t Robot1 Distance to Target\t "
                    "Robot2X \t Robot2Y \t Robot2V \t Robot2 Distance to Target\t "
                    "Robot3X \t Robot3Y \t Robot3V \t Robot3 Distance to Target\n";
    }
    if(m_myfile.is_open() && m_simulationOn)
    {
        qDebug()<<"PRINTING";
        auto t2 = std::chrono::high_resolution_clock::now();
        //! duration since starting time t1
        time = (t2 - t1);

        //! test if there is at least 1 robot to print the information of robot 1
        if(m_fishRobots.size()>0)
        {
            pos = m_fishRobots.at(0)->getPosition();
            vel = m_fishRobots.at(0)->getLinearVelocity();
            QPoint target = m_fishRobots.at(0)->getTargetPosition();
            distance = sqrt((pos.x()-target.x())*(pos.x()-target.x())+(pos.y()-target.y())*(pos.y()-target.y()));

            m_myfile<<method<<"\t"<<time.count()/1000<<"\t";
            m_myfile <<pos.x()<<"\t"<<pos.y()<<"\t"<<vel<<"\t"<<distance<<"\t";

        }
        //! test if there are at least 2 robots to print the information of robot 2
        if(m_fishRobots.size()>1)
        {
            pos = m_fishRobots.at(1)->getPosition();
            vel = m_fishRobots.at(1)->getLinearVelocity();
            QPoint target = m_fishRobots.at(1)->getTargetPosition();
            distance = sqrt((pos.x()-target.x())*(pos.x()-target.x())+(pos.y()-target.y())*(pos.y()-target.y()));

            m_myfile <<pos.x()<<"\t"<<pos.y()<<"\t"<<vel<<"\t"<<distance<<"\t";
        }
        //! test if there are at least 3 robots to print the information of robot 3
        if(m_fishRobots.size()>2)
        {
            pos = m_fishRobots.at(2)->getPosition();
            vel = m_fishRobots.at(2)->getLinearVelocity();
            QPoint target = m_fishRobots.at(2)->getTargetPosition();
            distance = sqrt((pos.x()-target.x())*(pos.x()-target.x())+(pos.y()-target.y())*(pos.y()-target.y()));

            m_myfile <<pos.x()<<"\t"<<pos.y()<<"\t"<<vel<<"\t"<<distance<<"\t";
        }
        m_myfile<<"\n";
    }
    */
}

//! this method stops the timer and pauses the simulation
void SwarmInterface::stopSimulation()
{
    m_timer.stop();
}

//! DEPRICATED
//! Code taken from : http://www.qtforum.org/article/31434/qgraphicsscene-PROPer-remove-of-the-items.html
void SwarmInterface::clearScene()
{
    QList<QGraphicsItem *> list = scene->items();
    QList<QGraphicsItem *>::Iterator it = list.begin();

    for ( ; it != list.end(); ++it )
    {
        if ( *it )
        {
            scene->removeItem(*it);
            delete *it;
        }
    }

    delete scene; // This line causes the code to crash, that is why the function is not called yet
    scene = new QGraphicsScene();

    /*
    ui->graphicsView->setScene(scene);
    */
}

void SwarmInterface::initializeScene()
{
    //clearScene(); //This function cannot be called until the issue is resolved
    m_configurationSpace.clear();

    // Get the dimensions of our simulation vire
    QSize size = ui->SimulationView->size();
    size.setWidth(size.width()-2);
    size.setHeight(size.height()-2);

    //Load the image and convert to RGB format
    m_imageObject = m_imageObject.convertToFormat(QImage::Format_RGB888);
    m_imageObject = m_imageObject.scaled(size, Qt::KeepAspectRatio,
                                     Qt::SmoothTransformation);

    //Save the image in the configuration space vector
    //For each pixel get the gray level and etermine the pixel state

    int i, j, grayLevel;

    for (i = 0; i < m_imageObject.width(); i++)
    {
        std::vector<State> row; // Create an empty row
        for (j = 0; j < m_imageObject.height(); j++)
        {
            /*
            // TO HAVE A WHITE BACKGROUND
            m_imageObject.setPixelColor(i, j, Qt::white);
            row.push_back(State::FREE); // Add an element to the row
            */

            grayLevel = qGray(m_imageObject.pixel(i,j));
            if (i== 0 || i == m_imageObject.width()-1
                || j == 0 || j == m_imageObject.height()-1)
            {
                m_imageObject.setPixel(i, j, QColor(Qt::black).rgb());
                row.push_back(State::OCCUPIED); // Add an element to the row
            }
            else if(grayLevel < 10){
                m_imageObject.setPixel(i, j, QColor(Qt::black).rgb());
                row.push_back(State::OCCUPIED); // Add an element to the row

            }
            else if(grayLevel >= 10 && grayLevel<240){
                m_imageObject.setPixel(i, j, QColor(Qt::darkRed).rgb());
                row.push_back(State::HALLWAY);
            }
            else {
                m_imageObject.setPixel(i, j, QColor(Qt::white).rgb());
                row.push_back(State::FREE);
            }
        }
        m_configurationSpace.push_back(row); // Add the row to the main vector
    }

    for (int x = 478 ; x<=522;x++)
        for(int y = 498 ; y<= 542; y++)
            m_imageObject.setPixel(x, y, QColor(Qt::green).rgb());

    double rad= 1;

    for (int i = 0; i<4;i++)
    {
       int xCoord[4]= {510, 478,489,521};
       int yCoord[4] = {541, 509,498,530};
        for (int x = xCoord[i]-rad; x<=xCoord[i]+rad;x++ )
            for (int y = yCoord[i]-rad; y<=yCoord[i]+rad;y++ )
                m_imageObject.setPixel(x, y, QColor(Qt::yellow).rgb());
    }


    //Transform the modified image into a pixmap that will be aded to the scene
    // and then to the graphics view
    m_imagePixmap = QPixmap::fromImage(m_imageObject);
    scene->setSceneRect(0, 0, size.width(), size.height());
    ui->SimulationView->setCacheMode(QGraphicsView::CacheBackground); //to speed up the rendering
    scene->addPixmap(m_imagePixmap);
    ui->SimulationView->setScene(scene);
    //--------

    //Give the targets the configuration space - to be modified based on how djikstra will be used
    Target::setConfigurationSpace(m_configurationSpace);
    initializeDijkstra();
    initializePotentialField();
    initializeDynamicWindow();

    //this function was placed here in the case the scene was deleted, position to be modified
    //depending on whether the issue is resolved
    QObject::connect(&m_timer, SIGNAL(timeout()),scene, SLOT(advance()));
}

//-----------------------------------------------------------------//
//---------Functions related to the Items in the Simulation--------//
//-----------------------------------------------------------------//

void SwarmInterface :: deleteAllObjects()
{
    while (!m_fishRobots.empty() && m_fishRobotsCount!=0)
    {
       m_fishRobotsCount--;
       scene->removeItem(m_targets[m_fishRobotsCount]);
       scene->removeItem(m_fishRobots[m_fishRobotsCount]);
       m_fishRobots.pop_back();
       m_targets.pop_back();
    }
}

void SwarmInterface :: positionFishRobots(int newFishCount)
{
    QPoint objectPos; //the positions will later have to be determined based
                         //on actual position of the robot

    //if there are less fishRobots then desired and new fishRobots and their corresponding
    //targets (TO BE DONE : REMOVE THE DJIKSTRA PATH)
    qDebug()<<"entering function : old "<<m_fishRobotsCount<<" new : "<<newFishCount;

    if( m_fishRobotsCount < newFishCount )
    {
        qDebug()<<"adding fish : old "<<m_fishRobotsCount<<" new : "<<newFishCount;
        while( m_fishRobotsCount != newFishCount )
        {
            m_fishRobotsCount++;

            m_targets.push_back(new Target);
            m_fishRobots.push_back(new FishRobot(m_targets[m_fishRobotsCount-1],m_fishRobotsCount-1));

            qDebug()<<"fishRobotID"<<m_fishRobotsCount-1;

            if (m_fishRobotsCount == 1)
            {
                objectPos.setX((int)(ui->x1->value()*m_scaleFactor));
                objectPos.setY(ui->y1->value()*m_scaleFactor);
            }
            if (m_fishRobotsCount == 2 )
            {
                objectPos.setX((int)(ui->x2->value()*m_scaleFactor));
                objectPos.setY((int)(ui->y2->value()*m_scaleFactor));
            }
            if (m_fishRobotsCount ==3 )
            {
                objectPos.setX((int)(ui->x3->value()*m_scaleFactor));
                objectPos.setY((int)(ui->y3->value()*m_scaleFactor));
            }

            //if the current pixel is OCCUPIED keep trying to reposition the
            //fish robot at random
            while (m_configurationSpace[objectPos.x()][objectPos.y()] == State::OCCUPIED)
            {
                objectPos.setX(550);//std::rand() % width);
                objectPos.setY(700);//std::rand() % height);
            }

            //Set the position of the fishRobots and the Target once everything is in order
            m_fishRobots[m_fishRobotsCount-1]->setPosition(objectPos); //store position
            m_fishRobots[m_fishRobotsCount-1]->setPos(objectPos); //set position in grpahics

            /*
            while (m_configurationSpace[objectPos.x()][objectPos.y()] == State::OCCUPIED)
            {
                objectPos.setX(200);//std::rand() % width);
                objectPos.setY(50);//::rand() % height);
            }

            m_targets[m_fishRobotsCount-1]->setPosition(objectPos);
            m_targets[m_fishRobotsCount-1]->setPos(objectPos);
            */

            //Add the fish robots and the target to the scene
            scene->addItem(m_fishRobots[m_fishRobotsCount-1]);
            scene->addItem(m_targets[m_fishRobotsCount-1]);

            //update the fish numbers in the combo box
            ui->DjikstraComboBox->addItem(QString::number(m_fishRobotsCount));
        }
    }

    //if there are more robots than we want, remove the fishrobot and the
    //corresponding target

    if(m_fishRobotsCount>newFishCount)
    {
        while (m_fishRobotsCount!= newFishCount)
        {
           m_fishRobotsCount--;
           scene->removeItem(m_fishRobots[m_fishRobotsCount]);
           scene->removeItem(m_targets[m_fishRobotsCount]);
           m_fishRobots.pop_back();
           m_targets.pop_back();
           scene->removeItem(m_pointPlacedFishRobots.at(m_fishRobotsCount));
           m_pointPlacedFishRobots.pop_back();

           while(!m_djikstraFishRobotsPoints.at(m_fishRobotsCount).empty())
           {
                scene->removeItem(m_djikstraFishRobotsPoints.at(m_fishRobotsCount).at(0));
                m_djikstraFishRobotsPoints.at(m_fishRobotsCount).erase(m_djikstraFishRobotsPoints.at(m_fishRobotsCount).begin());
           }

           //! update the fish numbers in the combo box
           ui->DjikstraComboBox->removeItem(m_fishRobotsCount+1);
        }
    }
    resizeDijkstra();
}

void SwarmInterface :: scaleFishRobots()
{
    //! Calculate the new robot dimensions in pixels
    float newRobotHeight = ui->RobotHeightDoubleSpinBox->value()*m_scaleFactor;
    float newRobotWidth = ui->RobotLengthDoubleSpinBox->value()*m_scaleFactor;

    //! Set the robot dimensions
    FishRobot::setFishRobotDimensions(newRobotWidth,newRobotHeight);
}

void SwarmInterface :: djikstraSetGoal(int index)
{
    double rad = 2;

    //! if outside the simulation window quit
    if ( m_goalFishRobots.at(index).x()>ui->SimulationView->width()
       || m_goalFishRobots.at(index).y()>ui->SimulationView->height()
       || m_goalFishRobots.at(index).x()< 0 || m_goalFishRobots.at(index).y()<0)
    {
        return;
    }

    //! if a point has been placed for the first fishRobot, remove it
    if (m_pointPlacedFishRobots.at(index))
    {
        scene->removeItem(m_pointPlacedFishRobots.at(index));
    }

    //! if the event coordinates is not in an OCCUPIED cell, place a green point
    if(m_configurationSpace.at(m_goalFishRobots.at(index).x()).at(m_goalFishRobots.at(index).y())!= State::OCCUPIED)
    {
        m_pointPlacedFishRobots.at(index) = new QGraphicsEllipseItem(m_goalFishRobots.at(index).x()-rad,
                                                         m_goalFishRobots.at(index).y()-rad,
                                                         rad*2.0, rad*2.0);
        m_pointPlacedFishRobots.at(index)->setBrush(*new QBrush(Qt::green));
        scene->addItem(m_pointPlacedFishRobots.at(index));
    }
    else (m_pointPlacedFishRobots.at(index) = NULL);
}

void SwarmInterface::newScaleFactor()
{
    //! Calculate the new Scale Factor
    float scale_den = (float) std::max(ui->ArenaHeightDoubleSpinBox->value(),
                               ui->ArenaLengthDoubleSpinBox->value());
    float scale_num = (float) std::max(ui->SimulationView->width(),
                               ui->SimulationView->height());
    m_scaleFactor = scale_num/scale_den;
}

void SwarmInterface::drawDjikstraFishRobot(int index)
{
    static int distNodes = (ui->DistanceNodes_spinbox->value()*m_scaleFactor);
    int pathType = ui->DijkstraPathComboBox->currentIndex();

    //! pause the simulation
    if(m_simulationOn)
    {
        stopSimulation();
        m_simulationOn = false ;
    }


    if (distNodes != (int)(ui->DistanceNodes_spinbox->value()*m_scaleFactor))
    {
        distNodes = ui->DistanceNodes_spinbox->value()*m_scaleFactor;
        initializeDijkstra();
    }


    //remove previous path from the scene
    while(!m_djikstraFishRobotsPoints.at(index).empty())
    {
        scene->removeItem(m_djikstraFishRobotsPoints.at(index).back());
        m_djikstraFishRobotsPoints.at(index).pop_back();
    }

    //get the start and goal coordinates
    QPoint startCoord, goalCoord;
    startCoord = m_fishRobots[index]->getPosition();
    goalCoord = m_targets[index]->getPosition();

    if (m_pointPlacedFishRobots.at(index)) //if a point has been placed for the fishRobot1
    {
        goalCoord.setX(m_goalFishRobots.at(index).x());
        goalCoord.setY(m_goalFishRobots.at(index).y());
    }

    //get the djikstra path for the first fishRobot
    m_djikstraFishRobotsPath.at(index) = m_djikstraFishRobots->getDijkstraPath(startCoord,goalCoord, (DijkstraPath)pathType);

    if (m_pointPlacedFishRobots.at(index) && m_djikstraFishRobotsPath.at(index).empty())
    {
        goalCoord = m_targets[index]->getPosition();
        m_djikstraFishRobotsPath.at(index) = m_djikstraFishRobots->getDijkstraPath(startCoord,goalCoord, (DijkstraPath)pathType);
    }

    //give the path to the fish robot
    m_fishRobots.at(index)->setDijkstraPath(m_djikstraFishRobotsPath.at(index));

    //draw the path
    double rad = 2;

    for (int i = 0; i<(int)m_djikstraFishRobotsPath.at(index).size(); i++)
    {
        int xCoord = m_djikstraFishRobotsPath.at(index).at(i).x();
        int yCoord = m_djikstraFishRobotsPath.at(index).at(i).y();

        QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(xCoord-rad, yCoord-rad, rad*2.0, rad*2.0);
        ellipse->setBrush(*new QBrush(Qt::blue));
        m_djikstraFishRobotsPoints.at(index).push_back(ellipse);
        scene->addItem(m_djikstraFishRobotsPoints.at(index).back());
    }
}


//! this method updates all the potential field parameters
void SwarmInterface::updatePotentialFieldParameters()
{
    int newArenaNu      = (ui->ArenaRepulsiveForce->value());
    int newArenaRho0    = (ui->ArenaRepulsiveDist->value()*m_scaleFactor);
    int newRobotsNu     = (ui->RobotsRepulsiveForce->value());
    int newRobotsRho0   = (ui->RobotRepulsiveDist->value()*m_scaleFactor);
    int newZeta         = (ui->attractiveForce->value());
    int newdGoalStar    = (ui->attractiveDist->value()*m_scaleFactor);
    int newMaxForce     = (ui->MaxForce->value());
    int newMaxAngle     = ui->InfluenceAngle->value();

    qDebug()<<"Arena : "<<newArenaNu<<newArenaRho0;
    qDebug()<<"Robots : "<<newRobotsNu<<newRobotsRho0;
    qDebug()<<"Target : "<<newZeta<<newdGoalStar;

    m_potentialField->setParameters(newRobotsNu, newRobotsRho0, newArenaNu, newArenaRho0, newZeta, newdGoalStar, newMaxForce, newMaxAngle);
}


//-----------------------------------------------------------------//
//------------Functions related to the Controls--------------------//
//-----------------------------------------------------------------//

void SwarmInterface::mousePressEvent(QMouseEvent * mouseEvent)
{
    //get the event coordinates in the scene
    int currentFishRobot = ui->DjikstraComboBox->currentText().toInt();
    QPoint goalFishRobot = ui->SimulationView->mapFromParent(mouseEvent->pos());

    if(currentFishRobot == 0)
    {
        int i;
        //set all the goals
        for(i = 0; i<m_fishRobotsCount;i++)
        {
            m_goalFishRobots.at(i) = goalFishRobot;
            djikstraSetGoal(i);
        }
    }
    else if (currentFishRobot <= m_fishRobotsCount)
    {
        m_goalFishRobots.at(currentFishRobot-1) = goalFishRobot;
        djikstraSetGoal(currentFishRobot-1);
    }
}

void SwarmInterface::on_StartButton_clicked()
{
    if (!m_simulationOn)
    {
        startSimulation();
        m_simulationOn = true;
    }
}

void SwarmInterface::on_PauseButton_clicked()
{
    if(m_simulationOn)
    {
        stopSimulation();
        m_simulationOn = false ;
    }    
}

void SwarmInterface::on_FishSpinBox_valueChanged(int newFishCount)
{
    positionFishRobots(newFishCount);
}

void SwarmInterface::on_LoadButton_clicked()
{
    QString imagePath = QFileDialog::getOpenFileName(
                    this,tr("Open File"),
                    "",
                    tr("PNG (*.png);; JPEG (*.jpg *.jpeg) ;; BMP (*.bmp)")
                    );
    //if an image has been chosen, reinitialize everything
    if(imagePath!=0)
    {
        m_imageObject.load(imagePath);
        initializeScene();
        deleteAllObjects();
        positionFishRobots(ui->FishSpinBox->value());
    }
}

void SwarmInterface::on_KpDoubleSpinBox_valueChanged(double newKp)
{
    FishRobot::setControllerParameters(Gains::PROP, newKp);
}

void SwarmInterface::on_KiDoubleSpinBox_valueChanged(double newKi)
{

    FishRobot::setControllerParameters(Gains::INTEG, newKi);
}

void SwarmInterface::on_KdDoubleSpinBox_valueChanged(double newKd)
{
    FishRobot::setControllerParameters(Gains::DERIV, newKd);
}

void SwarmInterface::on_LinearVelocityDoubleSpinBox_valueChanged(double newLinearVel)
{   newScaleFactor();
    FishRobot::setDesiredLinearVel(newLinearVel*m_scaleFactor);
}

void SwarmInterface::on_OmegaMaxDoubleSpinBox_valueChanged(double newOmegaMax)
{
    FishRobot::setOmegaMax(newOmegaMax);
}

void SwarmInterface::on_ArenaHeightDoubleSpinBox_valueChanged(double newArenaHeight)
{
      newScaleFactor();
      scaleFishRobots();
}

void SwarmInterface::on_ArenaLengthDoubleSpinBox_valueChanged(double newArenaLength)
{
      newScaleFactor();
      scaleFishRobots();
}

void SwarmInterface::on_RobotHeightDoubleSpinBox_valueChanged(double newRobotHeight)
{
    newScaleFactor();
    scaleFishRobots();
}

void SwarmInterface::on_RobotLengthDoubleSpinBox_valueChanged(double newRobotLength)
{
    newScaleFactor();
    scaleFishRobots();
}

void SwarmInterface::on_DJikstraDrawPath_clicked()
{
    int currentFishRobot = ui->DjikstraComboBox->currentText().toInt();
    int i;

    if(m_pathplanning == PathPlanning::DIJKSTRA
     ||m_pathplanning == PathPlanning::DIJKSTRADWA
     ||m_pathplanning == PathPlanning::DIJKSTRAPOTFIELD)
    {
        if (currentFishRobot>m_fishRobotsCount)
        {
            return;
        }

        if (currentFishRobot == 0)
        {
            for(i = 0; i<m_fishRobotsCount ; i++)
            {
                drawDjikstraFishRobot(i);
            }
            //DJIKSTRA FOR ALL THE FISHIES
            return;
        }
        else
        {
            //Djisktra for the fish chosen
            drawDjikstraFishRobot(currentFishRobot-1);
        }
    }
}

void SwarmInterface::on_PathPlanningComboBox_currentIndexChanged(int index)
{
    //pause the simulation
    if(m_simulationOn)
    {
        stopSimulation();
        m_simulationOn = false ;
    }

    m_pathplanning = (PathPlanning)ui->PathPlanningComboBox->currentIndex();
    FishRobot::setPathPlanningMethod(m_pathplanning);

    if (!(m_pathplanning == PathPlanning::DIJKSTRA
       || m_pathplanning == PathPlanning::DIJKSTRADWA
       || m_pathplanning == PathPlanning::DIJKSTRAPOTFIELD))
    {
        for (int i = 0 ; i<(int)m_djikstraFishRobotsPoints.size(); i++)
        {
            //remove previous path from the scene
            while(!m_djikstraFishRobotsPoints.at(i).empty())
            {
                scene->removeItem(m_djikstraFishRobotsPoints.at(i).back());
                m_djikstraFishRobotsPoints.at(i).pop_back();
            }
        }
    }
}

//! this method calls the update potential field parameters method when the attractive distance is changed
void SwarmInterface::on_attractiveDist_valueChanged(double arg1)
{
    updatePotentialFieldParameters();
}

//! this method calls the update potential field parameters method when the attractive force is changed
void SwarmInterface::on_attractiveForce_valueChanged(double arg1)
{
    updatePotentialFieldParameters();
}


//this method calls the update potential field parameters method when the angle of influence is changed
void SwarmInterface::on_InfluenceAngle_valueChanged(double arg1)
{
    updatePotentialFieldParameters();
}

//this method calls the update potential field parameters method when the max force value is changed
void SwarmInterface::on_MaxForce_valueChanged(double arg1)
{
    updatePotentialFieldParameters();
}

//this method calls the update potential field parameters method when the repulsive distance is changed
void SwarmInterface::on_RobotRepulsiveDist_valueChanged(double arg1)
{
    updatePotentialFieldParameters();
}
//this method calls the update potential field parameters method when the repulsive force is changed
void SwarmInterface::on_RobotsRepulsiveForce_valueChanged(double arg1)
{
    updatePotentialFieldParameters();
}

//this method calls the update potential field parameters method when the repulsive distance is changed
void SwarmInterface::on_ArenaRepulsiveDist_valueChanged(double arg1)
{
    updatePotentialFieldParameters();
}
//this method calls the update potential field parameters method when the repulsive force is changed
void SwarmInterface::on_ArenaRepulsiveForce_valueChanged(double arg1)
{
    updatePotentialFieldParameters();
}

void SwarmInterface::on_alpha_spinbox_valueChanged(int arg1)
{
    updateDWAParameters();
}

void SwarmInterface::on_beta_spinbox_valueChanged(int arg1)
{
    updateDWAParameters();
}

void SwarmInterface::on_gamma_spinbox_valueChanged(int arg1)
{
    updateDWAParameters();
}

void SwarmInterface::on_delta_spinbox_valueChanged(int arg1)
{
    updateDWAParameters();
}

void SwarmInterface::on_FinalTargetDistance_valueChanged(double arg1)
{
    //! set the admissible distances to the targets
    FishRobot::setAdmissibleTargetDistances(ui->FinalTargetDistance->value()*m_scaleFactor,
                                            ui->DijkstraIntermediateTargetDistance->value()*m_scaleFactor);
}

void SwarmInterface::on_DijkstraIntermediateTargetDistance_valueChanged(double arg1)
{
    //! set the admissible distances to the targets
    FishRobot::setAdmissibleTargetDistances(ui->FinalTargetDistance->value()*m_scaleFactor,
                                            ui->DijkstraIntermediateTargetDistance->value()*m_scaleFactor);
}

void SwarmInterface::on_DijkstraPathComboBox_currentIndexChanged(int index)
{
    FishRobot::setDijkstraPathType((DijkstraPath)index);
    on_DJikstraDrawPath_clicked(); //! FIXME : put a function
}

void SwarmInterface::on_QuitButton_clicked()
{
    if(m_myfile.is_open())
    {
        m_myfile.close();
    }
}


void SwarmInterface::on_DWAdistGoalLimit_valueChanged(double arg1)
{
    updateDWAParameters();
}

void SwarmInterface::on_DWAAngleLimit_valueChanged(const QString &arg1)
{
    updateDWAParameters();
}

void SwarmInterface::on_DWARobotDistLimit_valueChanged(double arg1)
{
    updateDWAParameters();
}

void SwarmInterface::on_DWAOccupiedRadiusLimit_valueChanged(double arg1)
{
    updateDWAParameters();
}

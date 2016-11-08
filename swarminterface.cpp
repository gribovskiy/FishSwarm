//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 6
//Last Modified :
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#include "swarminterface.h"
#include "ui_swarminterface.h"


#define INITIAL_FISH_COUNT 1

SwarmInterface::SwarmInterface(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SwarmInterface)
{
    ui->setupUi(this);

    //define the parameters of the view
    ui->SimulationView->setHorizontalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
    ui->SimulationView->setVerticalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );

    //load and image and set up the scene
    imageObject.load(":/Arenes/Images/arena_triang.png");
    scene = new QGraphicsScene();
    SwarmInterface_InitializeScene();

    //set up the fishRobots
    SwarmInterface_InitializeFishRobots();

    //Initialize the random function
    std::srand(std::time(0));

    SwarmInterface_StartSimulation();
}

SwarmInterface::~SwarmInterface()
{
    SwarmInterface_DeleteAllObjects();
    delete ui;
}

//-----------------------------------------------------------------//
//Functions related to the Initialisation of the Simulation
void SwarmInterface::SwarmInterface_InitializeFishRobots()
{
    //Set the correct dimensions for the fish robots
    SwarmInterface_ScaleFishRobots();

    //Set up the PID controller parameters
    FishRobot::setControllerParameters(PROP, ui->KpSpinBox->value());
    FishRobot::setControllerParameters(INTEG, ui->KiSpinBox->value());
    FishRobot::setControllerParameters(DERIV, ui->KdSpinBox->value());

    //Set the desired linear speed and max angular rotation
    FishRobot::setLinearVel(ui->LinearVelocitySpinBox->value());
    FishRobot::setOmegaMax(ui->OmegaMaxSpinBox->value());
    SwarmInterface_PositionFishRobots(INITIAL_FISH_COUNT);
}

void SwarmInterface::SwarmInterface_StartSimulation()
{   
    timer.start(1000 / 33); //equivalent to 30 frames/sec
    scene->update();
}

void SwarmInterface::SwarmInterface_StopSimulation()
{
    timer.stop();
}

//Code taken from : http://www.qtforum.org/article/31434/qgraphicsscene-proper-remove-of-the-items.html
void SwarmInterface::SwarmInterface_ClearScene()
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

void SwarmInterface::SwarmInterface_InitializeScene()
{
    //SwarmInterface_ClearScene(); //This function cannot be called until the issue is resolved
    configurationSpace.clear();

    // Get the dimensions of our simulation vire
    QSize size = ui->SimulationView->size();
    size.setWidth(size.width()-2);
    size.setHeight(size.height()-2);

    //Load the image and convert to RGB format
    imageObject = imageObject.convertToFormat(QImage::Format_RGB888);
    imageObject = imageObject.scaled(size, Qt::KeepAspectRatio,
                                     Qt::SmoothTransformation);

    //Save the image in the configuration space vector
    //For each pixel get the gray level and etermine the pixel state

    int i, j, grayLevel;

    for (i = 0; i < imageObject.width(); i++)
    {
        std::vector<int> row; // Create an empty row
        for (j = 0; j < imageObject.height(); j++)
        {
            /*
            imageObject.setPixelColor(i, j, Qt::white);
            row.push_back(FREE); // Add an element to the row
            */

            grayLevel = qGray(imageObject.pixel(i,j));

            if(grayLevel < 10){
                imageObject.setPixelColor(i, j, Qt::black);
                row.push_back(OCCUPIED); // Add an element to the row
            }
            else if(grayLevel >= 10 && grayLevel<240){
                imageObject.setPixelColor(i, j, Qt::darkRed);
                row.push_back(HALLWAY);
            }
            else {
                imageObject.setPixelColor(i, j, Qt::white);
                row.push_back(FREE);
            }
        }
        configurationSpace.push_back(row); // Add the row to the main vector
    }

    //Transform the modified image into a pixmap that will be aded to the scene
    // and then to the graphics view
    imagePixmap = QPixmap::fromImage(imageObject);
    scene->setSceneRect(0, 0, size.width(), size.height());
    ui->SimulationView->setCacheMode(QGraphicsView::CacheBackground); //to speed up the rendering
    scene->addPixmap(imagePixmap);
    ui->SimulationView->setScene(scene);
    //--------

    //Give the lures the configuration space - to be modified based on how djikstra will be used
    Lures::setConfigurationSpace(configurationSpace);

    //Set up the djikstra shortest path algorithm, currently for the first fish
    int distNodes = 20; // distance between the nodes, to be incorporated to the ui
    djikstraFishRobot1 = new DjikstraBoost(distNodes, configurationSpace);


    //this function was placed here in the case the scene was deleted, position to be modified
    //depending on whether the issue is resolved
    QObject::connect(&timer, SIGNAL(timeout()),scene, SLOT(advance()));
}

//-----------------------------------------------------------------//
//---------Functions related to the Items in the Simulation--------//
//-----------------------------------------------------------------//

void SwarmInterface :: SwarmInterface_DeleteAllObjects()
{
    while (!fishRobots.empty() && fishRobotsCount!=0)
    {
       fishRobotsCount--;
       scene->removeItem(lures[fishRobotsCount]);
       scene->removeItem(fishRobots[fishRobotsCount]);
       fishRobots.pop_back();
       lures.pop_back();
    }
}

void SwarmInterface :: SwarmInterface_PositionFishRobots(int newFishCount)
{
    QPoint objectPos; //the positions will later have to be determined based
                         //on actual position of the robot

    int width = ui->SimulationView->geometry().width();
    int height = ui->SimulationView->geometry().height();

    //if there are less fishRobots then desired and new fishRobots and their corresponding
    //lures (TO BE DONE : REMOVE THE DJIKSTRA PATH)
    if( fishRobotsCount < newFishCount )
    {
        while( fishRobotsCount != newFishCount )
        {
            fishRobotsCount++;
            lures.push_back(new Lures);
            fishRobots.push_back(new FishRobot(lures[fishRobotsCount-1]));

            //add the robots in a circle and 100 pixels from the center
            objectPos.setX((float)sin(1 - TWOPI/fishRobotsCount)*100 + width/2);
            objectPos.setY((float)cos(1- TWOPI/fishRobotsCount)* 100 + height/2);

            //while the current pixel is occupied keep trying to reposition the
            //fish robot at random
            while (configurationSpace[objectPos.x()][objectPos.y()] == OCCUPIED)
            {
                objectPos.setX(std::rand() % width);
                objectPos.setY(std::rand() % height);
            }

            //Set the position of the fishRobots and the Lures once everything is in order
            //PROBLEM : tried removing the setPosition and only using setPos but it crashed...
            fishRobots[fishRobotsCount-1]->setPosition(objectPos); //store position
            fishRobots[fishRobotsCount-1]->setPos(objectPos); //set position in grpahics

            while (configurationSpace[objectPos.x()][objectPos.y()] == OCCUPIED)
            {
                objectPos.setX(std::rand() % width);
                objectPos.setY(std::rand() % height);
            }

            lures[fishRobotsCount-1]->setPosition(objectPos);
            lures[fishRobotsCount-1]->setPos(objectPos);

            //Add the fish robots and the lure to the scene
            scene->addItem(fishRobots[fishRobotsCount-1]);
            scene->addItem(lures[fishRobotsCount-1]);
        }
    }

    //if there are more robots than we want, remove the fishrobot and the
    //corresponding lure

    if(fishRobotsCount>newFishCount)
    {
        while (fishRobotsCount!= newFishCount)
        {
           fishRobotsCount--;
           scene->removeItem(fishRobots[fishRobotsCount]);
           scene->removeItem(lures[fishRobotsCount]);
           fishRobots.pop_back();
           lures.pop_back();
        }
    }
}

void SwarmInterface :: SwarmInterface_ScaleFishRobots()
{
    //Calculate the new Scale Factor
    float scale_den = std::max(ui->ArenaHeightSpinBox->value(),
                               ui->ArenaLengthSpinBox->value());
    float scale_num = std::max(ui->SimulationView->width(),
                               ui->SimulationView->width());

    scaleFactor = scale_num/scale_den;

    //Calculate the new robot dimensions in pixels
    float newRobotHeight = ui->RobotHeightSpinBox->value()*scaleFactor;
    float newRobotWidth = ui->RobotLengthSpinBox->value()*scaleFactor;

    //Set the robot dimensions
    FishRobot::setFishRobotDimensions(newRobotWidth,newRobotHeight);
}

//-----------------------------------------------------------------//
//------------Functions related to the Controls--------------------//
//-----------------------------------------------------------------//

void SwarmInterface::mousePressEvent(QMouseEvent * mouseEvent)
{
    double rad = 2;

    //get the event coordinates in the scene
    goalFishRobot1 = ui->SimulationView->mapFromParent(mouseEvent->pos());

    //if a point has been placed for the first fishRobot, remove it
    if (pointPlacedFishRobot1)
    {
        scene->removeItem(pointPlacedFishRobot1);
    }

    //if the event coordinates is not in an occupied cell, place a green point
    if(configurationSpace.at(goalFishRobot1.x()).at(goalFishRobot1.y())!= OCCUPIED)
    {
        pointPlacedFishRobot1 = new QGraphicsEllipseItem(goalFishRobot1.x()-rad,
                                                         goalFishRobot1.y()-rad,
                                                         rad*2.0, rad*2.0);
        pointPlacedFishRobot1->setBrush(*new QBrush(Qt::green));
        scene->addItem(pointPlacedFishRobot1);
    }
    else (pointPlacedFishRobot1 = NULL);
}

void SwarmInterface::on_StartButton_clicked()
{
    if (!simulationOn)
    {
        SwarmInterface_StartSimulation();
        simulationOn = true;
    }
}

void SwarmInterface::on_PauseButton_clicked()
{
    if(simulationOn)
    {
        SwarmInterface_StopSimulation();
        simulationOn = false ;
    }    
}

void SwarmInterface::on_FishSpinBox_valueChanged(int newFishCount)
{
    SwarmInterface_PositionFishRobots(newFishCount);
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
        imageObject.load(imagePath);
        SwarmInterface_InitializeScene();
        SwarmInterface_DeleteAllObjects();
        SwarmInterface_PositionFishRobots(ui->FishSpinBox->value());
    }
}

void SwarmInterface::on_KpSpinBox_valueChanged(int newKp)
{
    FishRobot::setControllerParameters(PROP, (double)newKp/1000);
}

void SwarmInterface::on_KiSpinBox_valueChanged(int newKi)
{

    FishRobot::setControllerParameters(INTEG, (double)newKi/1000);
}

void SwarmInterface::on_KdSpinBox_valueChanged(int newKd)
{
    FishRobot::setControllerParameters(DERIV, (double)newKd/1000);
}

void SwarmInterface::on_LinearVelocitySpinBox_valueChanged(int newLinearVel)
{
    FishRobot::setLinearVel(newLinearVel);
}

void SwarmInterface::on_OmegaMaxSpinBox_valueChanged(int newOmegaMax)
{
    FishRobot::setOmegaMax(newOmegaMax);
}

void SwarmInterface::on_ArenaHeightSpinBox_valueChanged(int newArenaHeight)
{
      SwarmInterface_ScaleFishRobots();
}

void SwarmInterface::on_ArenaLengthSpinBox_valueChanged(int newArenaLength)
{
      SwarmInterface_ScaleFishRobots();
}

void SwarmInterface::on_RobotHeightSpinBox_valueChanged(int newRobotHeight)
{
    SwarmInterface_ScaleFishRobots();
}

void SwarmInterface::on_RobotLengthSpinBox_valueChanged(int newRobotLength)
{
    SwarmInterface_ScaleFishRobots();
}

void SwarmInterface::on_DJikstraDrawPathFish1_clicked()
{
    //pause the simulation
    if(simulationOn)
    {
        SwarmInterface_StopSimulation();
        simulationOn = false ;
    }

    //remove previous path from the scene
    while(!djikstraFishRobot1Points.empty())
    {
        scene->removeItem(djikstraFishRobot1Points.back());
        djikstraFishRobot1Points.pop_back();
    }

    //get the start and goal coordinates
    QPoint startCoord, goalCoord;
    startCoord = fishRobots[0]->getPosition();
    goalCoord = lures[0]->getPosition();

    if (pointPlacedFishRobot1) //if a point has been placed for the fishRobot1
    {
        goalCoord.setX(goalFishRobot1.x());
        goalCoord.setY(goalFishRobot1.y());
    }

    //get the djikstra path for the first fishRobot
    std::vector <std::pair<int,int> > djikstraFishRobot1Path = djikstraFishRobot1->getPath(startCoord,goalCoord);

    if (pointPlacedFishRobot1 && djikstraFishRobot1Path.empty())
    {
        goalCoord = lures[0]->getPosition();
        djikstraFishRobot1Path = djikstraFishRobot1->getPath(startCoord,goalCoord);
    }

    /*
    if (pointPlacedFishRobot1)
    {
        //if the shortest path goal node is not within the actual goal node cell
        if (djikstraFishRobot1Path.back().first() < goalCoord.x()+distNodes
                || djikstraFishRobot1Path.back().first() <goalCoord.x()-distNodes)
        {
            if (djikstraFishRobot1Path.back().second() >goalCoord.y()+distNodes
                    || djikstraFishRobot1Path.back().second() <goalCoord.y()-distNodes)
            {
                //compute djikstra with the lure coordinates
                goalCoord = lures[0]->getPosition();
                djikstraFishRobot1Path = djikstraFishRobot1->getPath(startCoord,goalCoord);
            }

        }
    }
    else
    {
        djikstraFishRobot1Path.clear();
    }
    */

    //draw the path
    int size = djikstraFishRobot1Path.size();
    double rad = 2;

    for (int i = 0; i<size; i++)
    {
        int xCoord = djikstraFishRobot1Path.at(i).first;
        int yCoord = djikstraFishRobot1Path.at(i).second;

        QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(xCoord-rad, yCoord-rad, rad*2.0, rad*2.0);
        ellipse->setBrush(*new QBrush(Qt::blue));
        djikstraFishRobot1Points.push_back(ellipse);
        scene->addItem(djikstraFishRobot1Points.back());
    }
}

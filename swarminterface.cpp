//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 5
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

    scene = new QGraphicsScene();
    //imageObject.load(":/Images/Images/arena/arena2Rooms_2Behaviors.bmp");
    imageObject.load(":/Images/Images/arena_triang.png");
    SwarmInterface_InitializeScene();
    SwarmInterface_ScaleFishRobots();

    /*
    scene->addPixmap(image);
    scene->setSceneRect(0,0,image.width(),image.height());
    ui->SimulationView->setCacheMode(QGraphicsView::CacheBackground); //to speed up the rendering
    ui->SimulationView->resize(image.width(), image.height()); //Verifier si nécessaire
    ui->SimulationView->setScene(scene);
    ui->SimulationView->show();
    */

    //INTIIALIZE EXPERIENCE PARAMETERS
    FishRobot::setControllerParameters(PROP, ui->KpSpinBox->value());
    FishRobot::setControllerParameters(INTEG, ui->KiSpinBox->value());
    FishRobot::setControllerParameters(DERIV, ui->KdSpinBox->value());
    FishRobot::setLinearVel(ui->LinearVelocitySpinBox->value());
    FishRobot::setOmegaMax(ui->OmegaMaxSpinBox->value());
    SwarmInterface_PositionFishRobots(INITIAL_FISH_COUNT);

    //INITIALIZE RANDOM FUNCTION
    std::srand(std::time(0));

    //QObject::connect(&timer, SIGNAL(timeout()),scene, SLOT(advance())); //INSERTED ELSEWHERE
    SwarmInterface_StartSimulation();


}

SwarmInterface::~SwarmInterface()
{
    SwarmInterface_DeleteAllObjects();
    delete ui;
}

//-----------------------------------------------------------------//
//Functions related to the Initialisation of the Simulation

void SwarmInterface::SwarmInterface_StartSimulation()
{   
    timer.start(1000 / 33); //equivalent to 30 frames/sec
    scene->update();
}

void SwarmInterface::SwarmInterface_StopSimulation()
{
    timer.stop();
}

//http://www.qtforum.org/article/31434/qgraphicsscene-proper-remove-of-the-items.html
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

    delete scene; // probleme!
    scene = new QGraphicsScene();
    /*
    scene = new QGraphicsScene();
    scene->setBackgroundBrush(QColor(0, 0, 0, 127));
    ui->graphicsView->setScene(scene);
    */
}

void SwarmInterface::SwarmInterface_InitializeScene()
{
    //SwarmInterface_ClearScene(); //-> ca crash avec cette ligne
    configurationSpace.clear();

    // Charge l'image, transforme en pixmap, ajoute la pixmap a la scene, ajoute la scene a la view
    QSize size = ui->SimulationView->size();
    imageObject = imageObject.convertToFormat(QImage::Format_RGB888);
    imageObject = imageObject.scaled(size, Qt::KeepAspectRatio, Qt::SmoothTransformation);

    //--------
    // Enregistrement des pixels de l'image //http://www.cplusplus.com/forum/general/833/

    int i, j, grayLevel;

    for (i = 0; i < imageObject.width(); i++)
    {
        std::vector<int> row; // Create an empty row
        for (j = 0; j < imageObject.height(); j++)
        {

            //QRgb pixelColor;
            //grayLevel = qGray(pixelColor);
            grayLevel = qGray(imageObject.pixel(i,j));

            /*
            if (imageObject.pixelColor(i,j) == Qt::blue)
            {
                imageObject.setPixelColor(i, j, Qt::darkRed);//QColor(0, 0, 255, 127));//semi-transparent blue
                row.push_back(GOSTRAIGHT);
            }
            if (imageObject.pixelColor(i,j) == Qt::white)
            {
                imageObject.setPixelColor(i, j, Qt::darkRed);//QColor(0, 0, 255, 127));//semi-transparent blue
                row.push_back(NORMAL);
            }
            if (imageObject.pixelColor(i,j) == Qt::black)
            {
                imageObject.setPixelColor(i, j, Qt::black);//QColor(0, 0, 255, 127));//semi-transparent blue
                row.push_back(FORBIDDEN);
            }
            else
            {
                imageObject.setPixelColor(i, j, Qt::green);//QColor(0, 0, 255, 127));//semi-transparent blue
                row.push_back(UNKNOWN);
            }
            */

            if(grayLevel < 10){
                imageObject.setPixelColor(i, j, Qt::black);
                row.push_back(FORBIDDEN); // Add an element (column) to the row
            }
            else if(grayLevel >= 10 && grayLevel<240){
                imageObject.setPixelColor(i, j, Qt::darkRed);
                row.push_back(GOSTRAIGHT);
            }
            else {
                imageObject.setPixelColor(i, j, Qt::white);
                row.push_back(NORMAL);
            }

            //qDebug()<<"i = "<<i<<" j = "<<j<<" : "<<row[j]<< " grayLevel : "<<grayLevel;

        }
        configurationSpace.push_back(row); // Add the row to the main vector
    }


    Lures::setConfigurationSpace(configurationSpace);
    //--------

    imagePixmap = QPixmap::fromImage(imageObject);
    scene->setSceneRect(0, 0, size.width(), size.height());
    ui->SimulationView->setCacheMode(QGraphicsView::CacheBackground); //to speed up the rendering
    scene->addPixmap(imagePixmap);
    ui->SimulationView->setScene(scene);

    //--------

    QObject::connect(&timer, SIGNAL(timeout()),scene, SLOT(advance()));
}

//-----------------------------------------------------------------//
//Functions related to the Items in the Simulation

void SwarmInterface :: SwarmInterface_DeleteAllObjects()
{

    while (!fishRobots.empty() && fishRobotsCount!=0)
    {
       fishRobotsCount--;
       scene->removeItem(lures[fishRobotsCount]);
       scene->removeItem(fishRobots[fishRobotsCount]);
    }
}

void SwarmInterface :: SwarmInterface_PositionFishRobots(int newFishCount)
{
    float pos[2]; //VOIR COMMENT RECUPERER LES VRAIES POSITIONS POUR LES POISSONS

    int width = ui->SimulationView->geometry().width();
    int height = ui->SimulationView->geometry().height();

    if( fishRobotsCount < newFishCount )
    {
        while( fishRobotsCount != newFishCount )
        {
            fishRobotsCount++;
            lures.push_back(new Lures);
            fishRobots.push_back(new FishRobot(lures[fishRobotsCount-1]));

            //INCORPORATE ACTUAL ROBOT POSITION WHEN INITIALIZING THE SIMULATION


            pos[0] = sin(1 - TWOPI/fishRobotsCount)*100 + width/2; //x
            pos[1] = cos(1- TWOPI/fishRobotsCount)* 100 + height/2; //y


            while (  configurationSpace[(int)pos[0]][(int)pos[1]]==FORBIDDEN)
            {
                pos[0] = std::rand() % width;
                pos[1] = std::rand() % height;
            }


            //SAVE POSITION COORDINATES IN MEMORY

            fishRobots[fishRobotsCount-1]->setPosition(pos); //store position
            fishRobots[fishRobotsCount-1]->setPos(pos[0],pos[1]); //set position in grpahics, check if redundant

            while (configurationSpace[pos[0]][pos[1]]==FORBIDDEN)
            {
                pos[0] = std::rand() % width;
                pos[1] = std::rand() % height;
            }

            lures[fishRobotsCount-1]->setPosition(pos); //same comments
            lures[fishRobotsCount-1]->setPos(pos[0],pos[1]);
            //POSITION ITEMPS IN GRAPHICS VIEW

            scene->addItem(fishRobots[fishRobotsCount-1]);
            scene->addItem(lures[fishRobotsCount-1]);
        }
    }

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

void SwarmInterface :: SwarmInterface_ScaleFishRobots() //see if better to dissociate into 2 functions.
{
    //Calculate the new Scale Factor
    float scale_den = std::max(ui->ArenaHeightSpinBox->value(), ui->ArenaLengthSpinBox->value());
    float scale_num = std::max(ui->SimulationView->width(), ui->SimulationView->width());

    scaleFactor = scale_num/scale_den;

    //Calculate the new robot dimensions in pixels
    float newRobotHeight = ui->RobotHeightSpinBox->value()*scaleFactor;
    float newRobotWidth = ui->RobotLengthSpinBox->value()*scaleFactor;

    FishRobot::setFishRobotDimensions(newRobotWidth,newRobotHeight);
}

//-----------------------------------------------------------------//
//Functions related to the Controls

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
    //int lure1Position[2];

    if(simulationOn)
    {
        SwarmInterface_StopSimulation();
        simulationOn = false ;
        /*
        if (!lures.empty())
        {
            Lures *lure1 = lures.front();
            Lures::getPosition(lure1, lure1Position);
            std::cout<<'x : '<<lure1Position[0]<<'\n';
            std::cout<<'y : '<<lure1Position[1]<<'\n';
        }
        */
    }    
}

void SwarmInterface::on_FishSpinBox_valueChanged(int newFishCount)
{
    SwarmInterface_PositionFishRobots(newFishCount);
}

void SwarmInterface::on_LoadButton_clicked()
{

    QString imagePath = QFileDialog::getOpenFileName(
                    this,
                    tr("Open File"),
                    "",
                    tr("JPEG (*.jpg *.jpeg);;PNG (*.png);; BMP (*.bmp)")
                    );
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

void SwarmInterface::on_ArenaHeightSpinBox_valueChanged(int arg1)
{
      SwarmInterface_ScaleFishRobots();
}

void SwarmInterface::on_ArenaLengthSpinBox_valueChanged(int arg1)
{
      SwarmInterface_ScaleFishRobots();
}

void SwarmInterface::on_RobotHeightSpinBox_valueChanged(int arg1)
{
    SwarmInterface_ScaleFishRobots();
}

void SwarmInterface::on_RobotLengthSpinBox_valueChanged(int arg1)
{
    SwarmInterface_ScaleFishRobots();
}

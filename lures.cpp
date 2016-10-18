//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 3
//Last Modified :
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#include "lures.h"


int state = DOWN ; //REMOVE
int speedRatio = 3, width = 550, height = 550;

Lures::Lures() : speed(100)
{
    setRotation(qrand() % (360 * 16)); //function inherited from QGraphics item
                                       //sets the initial orientation of the Lures
                                       //must be replaced by the actual orientation later on
}

/*
static qreal normalizeAngle(qreal angle)
{
    while (angle < 0)
        angle += TWOPI;
    while (angle > TWOPI)
        angle -= TWOPI;
    return angle;
}
*/

QRectF Lures::boundingRect() const
{
    qreal adjust = 0.5;
    return QRectF(-18 - adjust, -22 - adjust,
                  36 + adjust, 60 + adjust);
}

QPainterPath Lures::shape() const
{
    QPainterPath path;
    path.addRect(-10, -20, 20, 40);
    return path;
}

void Lures::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    // Body
    painter->setBrush((scene()->collidingItems(this).isEmpty() ? Qt::green : Qt::darkGreen));
    painter->drawEllipse(-4*ratio, -8*ratio, 4*ratio, 4*ratio);
}

//ADAPT TO THE SWARM ALGORITHM
void Lures::fix_out_of_bounds(int width, int height)
 {
    //IMPROVE THE CORRECTION BASED ON THE SIMULATION SPEED AND THE ROBOT SPEED VS THE MODEL SPEED
 }

void Lures::advance(int step = 1)//moves each Lures at each step of the program
{
    if (!step)
        return;

    counter++;

    if (counter!= 30)
        return;

    //CHECK THAT THE MOVEMENT OF THE LURES ARE INDEPENDANT... NOT SURE IT'LL WORK
    previousPos[0] = position[0];
    previousPos[1] = position[1];

    std::vector< std::vector<int> > configSpace = *(configurationSpace); //FIND A BETTER WAY TO DO THIS

    ///program the wall following...


    while( configSpace[ (int)position[0] ] [ (int)position[1] ] == FORBIDDEN)
    {
        // Random movement
        float theta = qrand() % 3;
        position[0] += speed*cos(theta)/30; //30fps
        position[1] += speed*sin(theta)/30; //30fps
    }

    position [0] = position[1] = 20;

    //Rotates the Item's Coordinate System by dx
    //setRotation(rotation() + theta);
    setPos(position[0], position[1]);

    /*
    switch(state){

    case down : position[1]+=1*speedRatio;
                if (position[1]>height-15)
                    state = right;
                break;
    case right : position[0]+=1*speedRatio;
                if (position[0]>width-15)
                    state = up;
        break;
    case up : position[1]-=1*speedRatio;
                if (position[1]<15)
                    state = left;
        break;
    case left : position[0]-=1*speedRatio;
                if(position[0]<30)
                    state = down;
    default :
        break;
    }




    WALL FOLLOWING ALGORITHM
    https://www.cs.hmc.edu/~dodds/projects/RobS01/Assignment2/Fixed_HTML/follow.html

    setPos(position[0],position[1]);
    */
}

void Lures::setPosition(int newPosition[2])
{
    previousPos[0] = position[0];
    previousPos[1] = position[1];

    position[0] = newPosition[0];
    position[1] = newPosition[1];
}

void Lures::getPosition(int currentPosition[2])
{
    currentPosition[0] = position[0];
    currentPosition[1] = position[1];
}

void Lures::setConfigurationSpace(std::vector< std::vector<int> > newConfigurationSpace)
{
     configurationSpace = new std::vector< std::vector<int> > (newConfigurationSpace);
}

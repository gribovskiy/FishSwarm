//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 3
//Last Modified :
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#include "lures.h"



int speedRatio = 5, width = 750, height = 750;

Lures::Lures() : m_speed(100)
{
    setRotation(qrand() % (360 * 16)); //function inherited from QGraphics item
                                       //sets the initial orientation of the Lures
                                       //must be replaced by the actual orientation later on
}

/*
static qreal normalizeangle(qreal angle)
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
    painter->drawEllipse(-4*m_ratio, -8*m_ratio, 4*m_ratio, 4*m_ratio);
}

//ADAPT TO THE SWARM ALGORITHM
void Lures::fix_out_of_bounds()
 {
    //IMPROVE THE CORRECTION BASED ON THE SIMULATION SPEED AND THE ROBOT SPEED VS THE MODEL SPEED
 }

void Lures::advance(int step = 1)//moves each Lures at each step of the program
{
    if (!step)
        return;

    /* DONT DELETE
    counter++;

    if (counter!= 30)
        return;
        */

    //CHECK THAT THE MOVEMENT OF THE LURES ARE INDEPENDANT... NOT SURE IT'LL WORK
    m_previousPos = m_position;

    std::vector<std::vector<State>> configSpace = *(configurationSpace); //FIND A BETTER WAY TO DO THIS

    ///program the wall following...


    /*
    while( configSpace[ (int)position[0] ] [ (int)position[1] ] == FORBIDDEN)
    {
        // Random movement
        float theta = qrand() % 3;
        position[0] += speed*cos(theta)/30; //30fps
        position[1] += speed*sin(theta)/30; //30fps
    }
    */

    //Rotates the Item's Coordinate System by dx
    //setRotation(rotation() + theta);

    m_position.setX(50);
    m_position.setY(200);

    /*
    switch(state){

    case DOWN : m_position.setY(m_position.y()+1*speedRatio);
                if (m_position.y()>height-30)
                    state = RIGHT;
                break;
    case RIGHT: m_position.setX(m_position.x()+1*speedRatio);
                if (m_position.x()>width-30)
                    state = UP;
        break;
    case UP : m_position.setY(m_position.y()-1*speedRatio);
                if (m_position.y()<30)
                    state = LEFT;
        break;
    case LEFT : m_position.setX(m_position.x()-1*speedRatio);
                if(m_position.x()<30)
                    state = DOWN;
    default :
        break;
    }
    */


    setPos(m_position);

/*
    WALL FOLLOWING ALGORITHM
    https://www.cs.hmc.edu/~dodds/projects/RobS01/Assignment2/Fixed_HTML/follow.html
*/

}

void Lures::setPosition(QPoint newPosition)
{
    m_previousPos = m_position;
    m_position = newPosition;
}

QPoint Lures::getPosition()
{
    return m_position;
}

void Lures::setConfigurationSpace(std::vector<std::vector<State>> newConfigurationSpace)
{
     configurationSpace = new std::vector<std::vector<State>> (newConfigurationSpace);
}

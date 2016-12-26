//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 3
//Last Modified :
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#include "lures.h"


//! FIXME :: get as input from simulator
int speedRatio = 5, width = 750, height = 750;

//----------------------------------------------------------------------------//
//-------------------------------Class Constructor----------------------------//
//----------------------------------------------------------------------------//

//! Class constructor. Instanciates the lure and sets the intial speed
Lures::Lures() : m_speed(100)
{

}


//----------------------------------------------------------------------------//
//------------------------- QGraphics Item Class Methods ---------------------//
//----------------------------------------------------------------------------//


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

//----------------------------------------------------------------------------//
//----------------------------- Non Exported Members -------------------------//
//----------------------------------------------------------------------------//

//---------------------------------------//
//------------Advance Methods------------//
//---------------------------------------//

//! Qgraphics advance method, this is called at each step of the program and
//! computes the new position of the target.
void Lures::advance(int step = 1)
{
    if (!step)
        return;



   // m_position.setX(50);
   // m_position.setY(200);



    /* DONT DELETE
    counter++;

    if (counter!= 30)
        return;
        */

    //CHECK THAT THE MOVEMENT OF THE LURES ARE INDEPENDANT... NOT SURE IT'LL WORK
    //m_previousPos = m_position;

    // std::vector<std::vector<State>> configSpace = *(configurationSpace); //FIND A BETTER WAY TO DO THIS

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
    setPos(m_position);
}

QPoint Lures::getPosition()
{
    return m_position;
}

void Lures::setConfigurationSpace(std::vector<std::vector<State>> newConfigurationSpace)
{
     configurationSpace = new std::vector<std::vector<State>> (newConfigurationSpace);
}

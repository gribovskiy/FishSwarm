//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 3
//Last Modified :
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation


#include "target.h"

//! FIXME :: get as input from simulator
int speedRatio = 5, width = 750, height = 750;


//----------------------------------------------------------------------------//
//-------------------------------Class Constructor----------------------------//
//----------------------------------------------------------------------------//

//! Class constructor. Instanciates the target and sets the intial speed
Target::Target(): m_speed(100)
{

}



//----------------------------------------------------------------------------//
//------------------------- QGraphics Item Class Methods ---------------------//
//----------------------------------------------------------------------------//

/*!
 * QGraphicsItem method, it returns the estimated area for the Target drawing
 */
QRectF Target::boundingRect() const
{
    qreal adjust = 0.5;
    return QRectF(-18 - adjust, -22 - adjust,
                  36 + adjust, 60 + adjust);
}

/*!
 * QGraphicsItem method, it returns returns the shape of our Target
 */
QPainterPath Target::shape() const
{
    QPainterPath path;
    path.addRect(-10, -20, 20, 40);
    return path;
}

/*!
 * QGraphicsItem method, paints the target
 */
void Target::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    // Body
    painter->setBrush((scene()->collidingItems(this).isEmpty() ? Qt::green : Qt::darkGreen));
    painter->drawEllipse(-4*m_drawingRatio, -8*m_drawingRatio, 4*m_drawingRatio, 4*m_drawingRatio);
}

//----------------------------------------------------------------------------//
//----------------------------- Non Exported Members -------------------------//
//----------------------------------------------------------------------------//

//---------------------------------------//
//------------Advance Methods------------//
//---------------------------------------//

//! Qgraphics advance method, this is called at each step of the program and
//! computes the new position of the target.
void Target::advance(int step = 1)
{
    if (!step)
        return;

    setPos(m_position);
}


//---------------------------------------//
//-------------Setter Methods------------//
//---------------------------------------//
/*!
 * Exported Member. Sets the target's position
 */
void Target::setPosition(QPoint newPosition)
{
    m_previousPos = m_position;
    m_position = newPosition;
    setPos(m_position);
}

/*!
 * Exported Member. Sets the new configuration space, not required at the
 * moment as the motion of the lures has not yet been implemented
 */
void Target::setConfigurationSpace(std::vector<std::vector<State>> newConfigurationSpace)
{
     m_configurationSpace = new std::vector<std::vector<State>> (newConfigurationSpace);
}

//---------------------------------------//
//-------------Getter Methods------------//
//---------------------------------------//

/*!
 * Exported Member. Gets the target's position
 */
QPoint Target::getPosition()
{
    return m_position;
}



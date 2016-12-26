//Autor : Laila El Hamamsy
//Date Created : Wednesday July 13th 2016
//Version : 3
//Last Modified :26.12.2016
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#include <qgraphicsitem.h>
#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>

#include <math.h>
#include <vector>
#include <stdbool.h>

#include "constants.h"

// FIXME : need a comment for every method (and for every class), for instance:


#ifndef TARGET_H
#define TARGET_H



/*!
 * Target Class . this class creates the targets for that the fishRobots will
 * follow. Their motion will be dictated by the zebra fish swarm algorithm that
 * is not yet perfected and has to be improved before incorporating into the
 * simulator.
 */

class Target : public QGraphicsItem
{
public:

    //-------------------------------------------//
    //-------------Class Constructor-------------//
    //-------------------------------------------//

    /*!
     * Class Constructor. It instanciates the targets.
     */
    Target();

    //-------------------------------------------//
    //--------------Exported Methods-------------//
    //-------------------------------------------//


    /*!
     * QGraphicsItem method, it returns the estimated area for the Target drawing
     */
    QRectF       boundingRect() const Q_DECL_OVERRIDE;
    /*!
     * QGraphicsItem method, it returns returns the shape of the Target
     */
    QPainterPath shape() const Q_DECL_OVERRIDE;
    /*!
     * QGraphicsItem method, paints the target
     */
    void         paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) Q_DECL_OVERRIDE;

    //-------------------------------------------//
    //-------------Setter Functions--------------//
    //-------------------------------------------//

    /*!
     * Exported Member. Sets the target's position
     */
    void         setPosition(QPoint newPosition);


    /*!
     * Exported Member. Sets the new configuration space, not required at the
     * moment as the motion of the lures has not yet been implemented
     */
    static void  setConfigurationSpace(std::vector<std::vector<State>> newConfigurationSpace);

    //-------------------------------------------//
    //-------------Getter Functions--------------//
    //-------------------------------------------//

    /*!
     * Exported Member. Gets the target's position
     */
    QPoint       getPosition();

protected slots:

    //! Qgraphics advance method, this is called at each step of the program and
    //! computes the new position of the target.
    void         advance(int step) Q_DECL_OVERRIDE; //handles the animation

private:
    //! speed of the target
    qreal        m_speed;
    //! previous and current position of the target. The previous position
    //! is not used but may come of use once the motion of the targets has
    //! been implemented
    QPoint       m_position, m_previousPos;
    //! Drawing ratio
    int          m_drawingRatio = 2;
    //-------------------------------------------//
    //----------Non Exported Methods-------------//
    //-------------------------------------------//

};

//! configuration space array, initialized to null and the same for all targets
static std::vector<std::vector<State>> *m_configurationSpace = NULL;

#endif // TARGET_H

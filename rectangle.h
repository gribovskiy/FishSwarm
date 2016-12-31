//Autor : Laila El Hamamsy
//Date Created : Sat December 31st 2016
//Version : 1
//Last Modified : 31.12.2016

#ifndef RECTANGLE_H
#define RECTANGLE_H


#include <QPoint>
#include <math.h>
#include <QVector2D>

//! this class creates rectangles and checks whether points are inside the bounds
//! of the reactangle as well as if 2 rectangles collide
class Rectangle
{
public:

    //! Class Constructor : this method creates a rectangle by giving the center of
    //! the rectangle, the width and height parameters as well as the orientation
    //! in the simulator's coordinate  system. A security margin can be added to
    //! the diagonal of the rectangle if desired
    Rectangle(QPoint center, float W, float H, float orientation, float addedMargin = 0);

    //! Exported member. This method returns 1 if the given point is inside the
    //! rectangle and 0 if it is not.
    //!http://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not
    int pointInRectangle(QPoint m);

    //! Exported member. This method implements a collision check done by seeing
    //! whether or not each corner of the rectangles is in the others.
    static bool collidingRectangles(Rectangle rect1, Rectangle rect2);

    QPoint A;
    QPoint B;
    QPoint C;
    QPoint D;

private:


    //! Non exported member. This method returns the dot product between two
    //! vectors
    int dot(QVector2D u, QVector2D v);
};

#endif // RECTANGLE_H

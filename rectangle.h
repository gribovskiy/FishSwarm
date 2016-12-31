#ifndef RECTANGLE_H
#define RECTANGLE_H


#include <QPoint>
#include <math.h>
#include <QVector2D>

class Rectangle
{
public:
    Rectangle(QPoint center, float W, float H, float orientation, float addedMargin = 0.01*750);

    //! Exported member. This method returns 1 if the given point is inside the
    //! rectangle and 0 if it is not.
    //!http://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not
    int pointInRectangle(QPoint m);

    //! Exported member. This method implements a collision check done by seeing
    //! whether or not each corner of the rectangles is in the others.
    static bool collidingRectangles(Rectangle rect1, Rectangle rect2);

private:
    QPoint A;
    QPoint B;
    QPoint C;
    QPoint D;

    //! Non exported member. This method returns the dot product between two
    //! vectors
    int dot(QVector2D u, QVector2D v);
};

#endif // RECTANGLE_H

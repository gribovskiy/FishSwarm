#include "rectangle.h"

Rectangle::Rectangle(QPoint center, float W, float H, float orientation, float addedMargin)//! FIXME input 1 cm from simulateur multiplied by scale factor
{
    QPoint A0(W, -H);
    QPoint B0(W, H);
    QPoint C0(-W, H);
    QPoint D0(-W, -H);

    //! transform to polar coordinates
    double diam = sqrt(W*W+H*H)/2+addedMargin;
    double thetaA, thetaB, thetaC, thetaD;

    thetaA = atan2(-(double)A0.x(),(double)A0.y());
    thetaB = atan2(-(double)B0.x(),(double)B0.y());
    thetaC = atan2(-(double)C0.x(),(double)C0.y());
    thetaD = atan2(-(double)D0.x(),(double)D0.y());

    //! project back to normal coordinates system and translate
    A.setX(center.x()+diam*sin(thetaA+(double)orientation));
    B.setX(center.x()+diam*sin(thetaB+(double)orientation));
    C.setX(center.x()+diam*sin(thetaC+(double)orientation));
    D.setX(center.x()+diam*sin(thetaD+(double)orientation));

    A.setY(center.y()-diam*cos(thetaA+(double)orientation));
    B.setY(center.y()-diam*cos(thetaB+(double)orientation));
    C.setY(center.y()-diam*cos(thetaC+(double)orientation));
    D.setY(center.y()-diam*cos(thetaD+(double)orientation));
}


//! Exported member. This method returns 1 if the given point is inside the
//! rectangle and 0 if it is not.
//!http://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not
int Rectangle::pointInRectangle(QPoint m)
{
    QVector2D AB(B-A);
    QVector2D AM(m-A);
    QVector2D BC(C-B);
    QVector2D BM (m-B);

    int dotABAM = dot(AB, AM);
    int dotABAB = dot(AB, AB);
    int dotBCBM = dot(BC, BM);
    int dotBCBC = dot(BC, BC);
    return 0 <= dotABAM && dotABAM <= dotABAB && 0 <= dotBCBM && dotBCBM <= dotBCBC;
}

//! Exported member. This method implements a collision check done by seeing
//! whether or not each corner of the rectangles is in the others.
bool Rectangle::collidingRectangles(Rectangle rect1, Rectangle rect2)
{
    //! test one by one if each corner of one rectangle is in the other if it is
    //! the case return true
    if(rect2.pointInRectangle(rect1.A) || rect2.pointInRectangle(rect1.B))
        return true;
    if(rect2.pointInRectangle(rect1.C) || rect2.pointInRectangle(rect1.D))
        return true;
    if(rect1.pointInRectangle(rect2.A) || rect1.pointInRectangle(rect2.B))
        return true;
    if(rect1.pointInRectangle(rect2.C) || rect1.pointInRectangle(rect2.D))
        return true;

    //! if no intersection
    return false;
}

//! Non exported member. This method returns the dot product between two
//! vectors
int Rectangle::dot(QVector2D u, QVector2D v)
{
    return u.x() * v.x() + u.y() * v.y();
}

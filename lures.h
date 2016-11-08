//Autor : Laila El Hamamsy
//Date Created : Wednesday July 13th 2016
//Version : 3
//Last Modified :
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#ifndef LURES_H
#define LURES_H

#include <qgraphicsitem.h>
#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>

#include <math.h>
#include <vector>
#include <stdbool.h>

#include "constants.h"

enum STATE{DOWN, RIGHT,UP,LEFT};

class Lures: public QGraphicsItem
{
    public:
        Lures();

        QRectF       boundingRect() const Q_DECL_OVERRIDE; //returns the estimated area for the Lures drawing
        QPainterPath shape() const Q_DECL_OVERRIDE; //returns the shape of our Lures
        void         paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) Q_DECL_OVERRIDE;
        void         setPosition(QPoint newPosition);
        QPoint       getPosition();
        static void  setConfigurationSpace(std::vector< std::vector<int> > newConfigurationSpace);
    protected slots:
        void         advance(int step) Q_DECL_OVERRIDE; //handles the animation

    private:
        qreal        speed;
        QPoint       position, previousPos;
        int          ratio = 2;
        void         fix_out_of_bounds(int width =550, int height =550); //FIX WIDTH AND HEIGHT
};

static std::vector< std::vector<int> > *configurationSpace = NULL;
static int counter = 0;

#endif // Lures_H

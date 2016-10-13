//Autor : Laila El Hamamsy
//Date Created : Tuesday July 12th 2016
//Version : 1
//Last Modified :
//Inspired from the Colliding Mice Example in the Qt 5.7 Documentation

#include "swarminterface.h"
#include "fishrobot.h"
#include <QApplication>
#include <QWidget>
#include <iostream>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SwarmInterface Main;
    Main.show();
    return a.exec();
}

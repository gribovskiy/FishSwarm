//Autor : Laila El Hamamsy
//Date Created : September 26th 2016
//Version : 4
//Last Modified :

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <math.h>

const float DEG2RAD = M_PI/180;
const float RAD2DEG = 180/M_PI;
const float simulation_dt      = 0.033;

enum class Gains        : int {PROP, INTEG, DERIV};
enum class State        : int {FREE, HALLWAY, OCCUPIED};
enum class PathPlanning : int {PID, DJIKSTRA, DJIKSTRADWA, POTFIELD};
enum class Approach     : int {LOCAL, GLOBAL};

#define sgn(x) ( x != 0 ? abs(x) / x : 0 ) //Pris du Code de WheeledRobot.cpp


#endif // CONSTANTS_H

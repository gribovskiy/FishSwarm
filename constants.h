//Autor : Laila El Hamamsy
//Date Created : September 26th 2016
//Version : 4
//Last Modified : 26.12.2016

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <math.h>
#include <QPoint>

//! constant to convert from degrees to radians
const float DEG2RAD = M_PI/180;

//! constant to convert from radians to degrees
const float RAD2DEG = 180/M_PI;

//! timestep of the simulation
const float simulation_dt      = 0.033;

//! enum class for PID parameter tuning
enum class Gains        : int {PROP, INTEG, DERIV};

//! enum class for the states of the configuration space
enum class State        : int {FREE, HALLWAY, OCCUPIED};

//! enum class for the path planning algorithm chosen
enum class PathPlanning : int {PID, DIJKSTRA, DIJKSTRADWA, POTFIELD, DIJKSTRAPOTFIELD};

//! enum class for the approach chosen for potential field
enum class Approach     : int {LOCAL, GLOBAL};

//! enum class for the strategy chosen for the priority planning
enum class Strategy     : int {CLOSEST, FARTHEST};

//! enum class to identify fish robot status

enum class FishBotStatus : int {TARGET_REACHED, BLOCKED, MOVING};

#define sgn(x) ( x != 0 ? abs(x) / x : 0 ) //Pris du Code de WheeledRobot.cpp


#endif // CONSTANTS_H

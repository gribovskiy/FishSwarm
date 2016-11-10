//Autor : Laila El Hamamsy
//Date Created : September 26th 2016
//Version : 4
//Last Modified :

#ifndef CONSTANTS_H
#define CONSTANTS_H

#define PI      3.14159265358979323846264338327950288419717 // FIXME : this constant is already defined in cmath
#define TWOPI   (2.0 * PI) // FIXME : this variable is used only twice => no need
#define DEG2RAD (PI/180) // FIXME : better use constansts than macros
#define RAD2DEG (180/PI) // FIXME : better use constansts than macros

#define INF     10000   // FIXME : not clean, you can use numeric_limits<float>::max

enum Gains {PROP, INTEG, DERIV}; // FIXME : better use enum class
enum Behaviour {FREE, HALLWAY, OCCUPIED}; // FIXME : better use enum class

#endif // CONSTANTS_H

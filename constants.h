//Autor : Laila El Hamamsy
//Date Created : September 26th 2016
//Version : 4
//Last Modified :


#ifndef CONSTANTS_H
#define CONSTANTS_H

#define PI      3.14159265358979323846264338327950288419717
#define TWOPI   (2.0 * PI)
#define DEG2RAD (PI/180)
#define RAD2DEG (180/PI)

#define INF     10000

enum Gains {PROP, INTEG, DERIV};
enum Behaviour {FREE, HALLWAY, OCCUPIED};

#endif // CONSTANTS_H

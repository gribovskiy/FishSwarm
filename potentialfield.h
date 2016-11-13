#ifndef POTENTIALFIELD_H
#define POTENTIALFIELD_H


class PotentialField
{
public:
    PotentialField();

    //Static Members:
    static void computeGlobalRepulsiveForce();

    //Non Static Members:
    void computeLocalRepulsiveForce();
    void computeAttractiveForce();

};

#endif // POTENTIALFIELD_H

//Autor : Laila El Hamamsy
//Date Created : Sun October 16th 2016
//Version : 1
//Last Modified :


#ifndef DJIKSTRA_H
#define DJIKSTRA_H

#include <QDebug>

#include <vector>
#include <stdbool.h>
#include <math.h>


#include "constants.h"

struct NODE;

class Djikstra
{
private :
<<<<<<< HEAD
    void setConfigurationSpace(std::vector< std::vector<int> > newConfigurationSpace, int distNodes);
=======
    void imageToVertexList(std::vector< std::vector<int>> newConfigurationSpace, int distNodes);
>>>>>>> f96a97ec7fd071b8af3b3610d69a7fdcd56473e9
    void computeDjikstraShortestPathAlgorithm(int startCoord[2], int goalCoord[2], int distNodes);
    void getGraphFromNodeList();   
    void searchForShortestPath();
    void reconstructPath();
    NODE* searchCorrespondingNode(int searchCoordX, int searchCoordY);


    bool noPath = false;

public:
    Djikstra(int startCoord[2], int goalCoord[2], int distNodes, std::vector< std::vector<int> > newConfigurationSpace);
    Djikstra(int startCoord[2], int goalCoord[2], int distNodes);

    std::vector<std::pair <int,int> > getPath(int distNodes);
};




#endif // DJIKSTRA_H

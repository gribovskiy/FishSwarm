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

    void getGraphFromNodeList();
    void setConfigurationSpace(std::vector< std::vector<int>> newConfigurationSpace, int distNodes);
    NODE* searchCorrespondingNode(int searchCoordX, int searchCoordY);
    void searchForShortestPath();
    void reconstructPath();


    NODE *startNode;
    NODE *goalNode;
    std::vector<NODE> *unvisitedNodes;
    std::vector<NODE> *shortestPath;
    std::vector< std::vector<int> > *configurationSpace = NULL;
    bool noPath = false;

public:
     Djikstra(int startCoord[2], int goalCoord[2], std::vector< std::vector<int> > newConfigurationSpace, int distNodes);
     std::vector<std::pair <int,int>> getPath();
};




#endif // DJIKSTRA_H

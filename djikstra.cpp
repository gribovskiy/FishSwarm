//Autor : Laila El Hamamsy
//Date Created : Sun October 16th 2016
//Version : 1
//Last Modified :

#include "djikstra.h"
#include <QDebug>
#include <iostream>

struct NODE{
    int     parentNodeCoord[2] = {INF,INF};
    float   distanceToStart;
    int     coordX;
    int     coordY;
    std::vector<NODE> neighborsList;

    NODE (int initCoordX, int initCoordY, float initDistanceToStart){
        coordX = initCoordX;
        coordY = initCoordY;
        distanceToStart = initDistanceToStart;
    }
};

NODE startNode(INF, INF, INF);
NODE goalNode(INF, INF, INF);
std::vector<NODE> unvisitedNodes, allNodes;
std::vector<NODE> shortestPath;
std::vector< std::vector<int> > configurationSpace;

Djikstra::Djikstra(int startCoord[2], int goalCoord[2], int distNodes, std::vector< std::vector<int> > newConfigurationSpace)
{
    allNodes.clear();
    configurationSpace.clear();

    if (distNodes == 0)
    {
        return;
    }

    if (distNodes % 2 == 0)
    {
        distNodes--;
    }

    setConfigurationSpace(newConfigurationSpace, distNodes);
    computeDjikstraShortestPathAlgorithm(startCoord, goalCoord, distNodes);
}

Djikstra::Djikstra(int startCoord[2], int goalCoord[2], int distNodes)
{
    computeDjikstraShortestPathAlgorithm(startCoord, goalCoord, distNodes);
}

void Djikstra::computeDjikstraShortestPathAlgorithm(int startCoord[2], int goalCoord[2], int distNodes)
{
    shortestPath.clear();
    noPath = false;

    int startX = startCoord[0]/distNodes;
    int startY = startCoord[1]/distNodes;
    startNode = searchCorrespondingNode (startX, startY);
    if (startNode.coordX ==INF && startNode.coordY == INF)
    {
        allNodes.push_back(startNode);
    }
    startNode.distanceToStart = 0;

    int goalX = goalCoord[0]/distNodes;
    int goalY = goalCoord[1]/distNodes;
    goalNode  = searchCorrespondingNode (goalX, goalY);
    if (goalNode.coordX ==INF && goalNode.coordY == INF)
    {
        allNodes.push_back(goalNode);
    }

    qDebug()<<" start get Graph";
    getGraphFromNodeList();
    qDebug()<<" end get Graph";

    unvisitedNodes.clear(); //not sure if necessary
    unvisitedNodes = allNodes;

    //searches for the shortest path while the unvisited list is not empty
    searchForShortestPath();
    qDebug()<<" end search shortest path";

    reconstructPath();
    qDebug()<<" end reconstruct path";
}

void Djikstra::reconstructPath()
{
    NODE currentNode = goalNode;

    if (!noPath)
    {
        //add the goal to the path
        shortestPath.push_back(currentNode);

        //add parents until reach the startNode
        while((currentNode.coordX != startNode.coordX || currentNode.coordY != startNode.coordY))
        {
            currentNode = searchCorrespondingNode(currentNode.parentNodeCoord[0],
                                                  currentNode.parentNodeCoord[1]);
            shortestPath.push_back(currentNode);
        }
    }
}

std::vector<std::pair <int,int>> Djikstra::getPath()
{
    std::vector<std::pair <int,int>> path;
    int numberNodes = std::distance(shortestPath.begin(),shortestPath.end());
    int i;

    for (i = 0; i<numberNodes; i++)
    {
        path.push_back(std::make_pair(shortestPath.at(i).coordX, shortestPath.at(i).coordY));
    }
    return path;
}

void Djikstra::searchForShortestPath()
{
    std::vector<NODE>::iterator unvisitedListIterator, removeIterator, neighborsListIterator;
    NODE currentNode = unvisitedNodes.at(0);

    int parentIndex;

    if (startNode.coordX == INF && startNode.coordY == INF)
    {
        noPath=true;
        std::cout<<"start out of bounds \n ";
        return;
    }
    if (goalNode.coordX == INF && goalNode.coordY == INF)
    {
        noPath=true;
        std::cout<<"goal out of bounds \n ";
        return;
    }

    while(!unvisitedNodes.empty())
    {
        //search for the node with the shortest distance from the start
        for(unvisitedListIterator = unvisitedNodes.begin();
            unvisitedListIterator!= unvisitedNodes.end(); unvisitedListIterator++)
        {
            int index = std::distance( unvisitedNodes.begin(), unvisitedListIterator);

            if(currentNode.distanceToStart < unvisitedNodes.at(index).distanceToStart)
            {
                 currentNode = unvisitedNodes.at(index);

                 if(currentNode.coordX != goalNode.coordX
                 || currentNode.coordY != goalNode.coordY)
                 {
                     return;
                 }            
                 removeIterator = unvisitedListIterator;
                 parentIndex = index;
            }
        }


        {
            //for all the nodes adjacent to the current node

            for(neighborsListIterator = currentNode.neighborsList.begin();
                neighborsListIterator!= currentNode.neighborsList.end(); neighborsListIterator++)
            {
                int index = std::distance(neighborsListIterator, currentNode.neighborsList.begin());
                int deltaCoordX = currentNode.coordX - currentNode.neighborsList.at(index).coordX;
                int deltaCoordY = currentNode.coordY - currentNode.neighborsList.at(index).coordY;

                float distNeighbor= sqrt(pow(deltaCoordX,2) + pow(deltaCoordY,2));
                float totalDistance = currentNode.distanceToStart + distNeighbor;

                if(currentNode.neighborsList.at(index).distanceToStart >totalDistance)
                {
                    currentNode.neighborsList.at(index).distanceToStart = totalDistance;
                    currentNode.neighborsList.at(index).parentNodeCoord[0] = unvisitedNodes.at(parentIndex).parentNodeCoord[0];
                    currentNode.neighborsList.at(index).parentNodeCoord[1] = unvisitedNodes.at(parentIndex).parentNodeCoord[1];
                }
            }
            //remove currentNode from list
            unvisitedNodes.erase(removeIterator);
        }

        //if no path is found
        if (unvisitedNodes.empty()
           && (currentNode.coordX != goalNode.coordX || currentNode.coordY != goalNode.coordY))
        {
            noPath = true;
        }
    }
}

NODE Djikstra::searchCorrespondingNode(int searchCoordX, int searchCoordY)
{
    std::vector<NODE>::iterator listIndex;

    for(listIndex = allNodes.begin();
        listIndex!= allNodes.end(); listIndex++)
    {
        int index = std::distance( allNodes.begin(), listIndex);

        int coordX = allNodes.at(index).coordX;
        int coordY = allNodes.at(index).coordY == searchCoordY;
        if(coordX == searchCoordX && coordY == searchCoordY)
        {
            return allNodes.at(index);
        }
    }

    NODE   noNode(INF,INF,INF);
    return noNode;
}

void Djikstra::getGraphFromNodeList()
{
    std::vector<std::vector<int>>:: iterator columnIterator, i;
    std::vector<int>::iterator rowIterator, j;

    std::vector<NODE>::iterator listIndex;

    NODE newNode(INF,INF, INF);
    int m = 0, n = 0;
    int k = 0, l = 0;

    for(listIndex = allNodes.begin();
        listIndex!= allNodes.end(); listIndex++)
    {
        int index = std::distance( allNodes.begin(), listIndex);
        m = allNodes.at(index).coordX;
        n = allNodes.at(index).coordY;

        for (k = m -1 ; k <m+1 ; k++)
        {
            for (l = n -1 ; l < n+1; l++)
            {
                if (k != m && l != n //to avoid adding the node as it's neighboor
                && k >= 0 && l >= 0
                && k< std::distance(configurationSpace.end(),configurationSpace.begin()) //to avoid accessing a point outside the configuration space
                && l< std::distance(configurationSpace.end()->end(),configurationSpace.begin()->begin())) //same
                {
                    if( configurationSpace.at(k).at(l) != FORBIDDEN)
                    {
                        NODE newNeighbor = searchCorrespondingNode(m, n);
                        //add the neighboors to the neighboorList with associated distance
                        if (newNode.coordX != INF && newNode.coordY != INF)
                        {

                            allNodes.at(index).neighborsList.push_back(newNeighbor);
                        }
                    }
                }
            }
        }
    }
}

void Djikstra::setConfigurationSpace(std::vector< std::vector<int> > newConfigurationSpace, int distNodes)
{
    std::vector<std::vector<int>>:: iterator columnIterator, k;
    std::vector<int>::iterator rowIterator, l;

    //INITIALISER START ET GOAL ICI!
    int m = 0, n = 0;
    int nodeState = NORMAL, step;

    if (distNodes == 1)
    {
         configurationSpace =  newConfigurationSpace;
    }

    step = distNodes/2;

    for( columnIterator = newConfigurationSpace.begin() + step;
         columnIterator + distNodes + step < newConfigurationSpace.end();
         columnIterator += distNodes, m++)
    {
        std::vector<int> row;

        int column = -std::distance(columnIterator, newConfigurationSpace.begin());
        qDebug()<<"column : "<<column;

        /*
        qDebug()<<std::distance(newConfigurationSpace.begin(),newConfigurationSpace.end());
        qDebug()<<std::distance(newConfigurationSpace.begin(),columnIterator);
        qDebug()<<"distNodes : "<<distNodes;
        qDebug()<<"m :"<<m;
        */

        for( rowIterator =  (*columnIterator).begin()+step;
             rowIterator +  distNodes + step < (*columnIterator).end();
             rowIterator += distNodes, n++)
        {           

            //test the cells around de i, j to determine state : normal, gostraight or forbidden
            for (k = columnIterator-step ; k<columnIterator+step; k++)
            {
                for (l = rowIterator-step; l<rowIterator+step ; l++)
                {
                    switch (*l)
                    {
                    case NORMAL : //because initialized NORMAL
                        break;

                    case GOSTRAIGHT :
                        if (nodeState == NORMAL)
                        {
                            nodeState = GOSTRAIGHT;
                        }
                        break;

                    case FORBIDDEN :
                        nodeState = FORBIDDEN;
                        break;

                    default :
                        break;
                    }
                }
            }
            row.push_back(nodeState);

            //qDebug()<<"state : "<<nodeState;

            if (nodeState != FORBIDDEN)
            {
                NODE newNode(m,n,INF);
                int rowN = -std::distance(columnIterator, newConfigurationSpace.begin());
                qDebug()<<"row : "<<rowN;
                qDebug()<<"m : "<<m<<" n : "<<n;
                qDebug()<<"x : "<<newNode.coordX<<" y : "<<newNode.coordY;

                allNodes.push_back(newNode);
            }
            nodeState = NORMAL; //reinitialize

        }
        n = 0;
        configurationSpace.push_back(row);
    }
}

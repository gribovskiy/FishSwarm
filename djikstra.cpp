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
    std::vector<std::pair <int,int>> neighborsList;

    NODE (int initCoordX, int initCoordY, float initDistanceToStart){
        coordX = initCoordX;
        coordY = initCoordY;
        distanceToStart = initDistanceToStart;
    }
};

NODE *startNode;
NODE *goalNode;
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

    qDebug()<<"startX : "<<startX<<" startY : "<<startY;

    startNode = searchCorrespondingNode (startX, startY);

    if (!startNode)
    {
        startNode->coordX = startX ;
        startNode->coordY = startY ;
        allNodes.push_back(*startNode) ;
    }
    qDebug()<<"Node start :"<<startNode->coordX<<", "<<startNode->coordY;
    startNode->distanceToStart = 0;

    int goalX = goalCoord[0]/distNodes;
    int goalY = goalCoord[1]/distNodes;
    goalNode  = searchCorrespondingNode (goalX, goalY);
    if (!goalNode)
    {
        goalNode->coordX = goalX ;
        goalNode->coordY = goalY ;
        allNodes.push_back(*goalNode) ;
    }

    //qDebug()<<" start get Graph";
    getGraphFromNodeList();
    //qDebug()<<" end get Graph";

    unvisitedNodes.clear(); //not sure if necessary
    unvisitedNodes = allNodes;
    qDebug()<<" start search shortest path";
    //searches for the shortest path while the unvisited list is not empty
    searchForShortestPath();
    //qDebug()<<" end search shortest path";

    qDebug()<<" start reconstruct path";
    reconstructPath();
    qDebug()<<" end reconstruct path";
}

void Djikstra::reconstructPath()
{
    NODE *currentNode = goalNode;

    qDebug()<<" inside reconstruct path function";
    qDebug()<<"goal Node : x "<<goalNode->coordX<<" y"<<goalNode->coordY;

    if (!noPath)
    {
        //add the goal to the path
        shortestPath.push_back(*currentNode);
         qDebug()<<"current Node : x "<<currentNode->coordX<<" y"<<currentNode->coordY;
        //add parents until reach the startNode
        while(currentNode->coordX != startNode->coordX && currentNode->coordY != startNode->coordY)
        {
            //qDebug()<<"inside while";
            currentNode = searchCorrespondingNode(currentNode->parentNodeCoord[0],
                                                  currentNode->parentNodeCoord[1]);
            //qDebug()<<"x : "<<currentNode->coordX<<"y : "<<currentNode->coordY;
            shortestPath.push_back(*currentNode);
        }
    }
    qDebug()<<" end reconstruct path function";
}

std::vector<std::pair <int,int>> Djikstra::getPath(int distNodes)
{
    std::vector<std::pair <int,int>> path;
    int numberNodes = std::distance(shortestPath.begin(),shortestPath.end());
    int i;

    for (i = 0; i<numberNodes; i++)
    {
        int coordX = distNodes*shortestPath.at(i).coordX;
        int coordY = distNodes*shortestPath.at(i).coordY;
        path.push_back(std::make_pair(coordX, coordY));
    }
    return path;
}

void Djikstra::searchForShortestPath()
{
    std::vector<NODE>::iterator removeIterator, unvisitedListIterator;
    std::vector<std::pair <int,int>>:: iterator neighborsListIterator;
    NODE* currentNode = &unvisitedNodes.at(0);

    qDebug()<<" entering  search function";

    if (startNode->coordX == INF && startNode->coordY == INF)
    {
        noPath=true;
        qDebug()<<"start out of bounds \n ";
        return;
    }
    if (goalNode->coordX == INF && goalNode->coordY == INF)
    {
        noPath=true;
        qDebug()<<"goal out of bounds \n ";
        return;
    }

    while(!unvisitedNodes.empty())
    {
        qDebug()<<" start searching through list";

        //search for the node with the shortest distance from the start
        for(unvisitedListIterator = unvisitedNodes.begin();
            unvisitedListIterator!= unvisitedNodes.end(); unvisitedListIterator++)
        {
            int index = std::distance(unvisitedNodes.begin(), unvisitedListIterator);

            if(currentNode->distanceToStart > unvisitedNodes.at(index).distanceToStart)
            {
                currentNode = &unvisitedNodes.at(index);
                removeIterator = unvisitedListIterator; //to remoive the current node from the unvisited list
                qDebug()<<"found shortest distance from start";
            }
        }
        int test = std::distance(unvisitedNodes.begin(), removeIterator);
        qDebug()<<"index to remove "<<test;
        //remove currentNode from the list
        unvisitedNodes.erase(removeIterator);
        
        qDebug()<<" start looking at neighbors";
        //look through the current Node's neighbors
        for(neighborsListIterator = currentNode->neighborsList.begin();
            neighborsListIterator!= currentNode->neighborsList.end();
            neighborsListIterator++)
        {
            int index     = std::distance(neighborsListIterator,
                                          currentNode->neighborsList.begin());
            int neighborX = currentNode->neighborsList.at(index).first;
            int neighborY = currentNode->neighborsList.at(index).second;

            NODE* currentNeighbor = searchCorrespondingNode(neighborX, neighborY);
            //compute the distance between the current node and it's neighbor

            int   deltaCoordX   = currentNode->coordX - neighborX;
            int   deltaCoordY   = currentNode->coordY - neighborY;
            float distNeighbor  = sqrt(pow(deltaCoordX,2) + pow(deltaCoordY,2));
            
            //compute the total distance from the start going through the current node
            float totalDistance = currentNode->distanceToStart + distNeighbor;
            
            //identify whether the current node is the parent of one of it's neighbors
            if(currentNeighbor->distanceToStart >totalDistance)
            {
                currentNeighbor->distanceToStart    = totalDistance;
                currentNeighbor->parentNodeCoord[0] = currentNode->coordX;
                currentNeighbor->parentNodeCoord[1] = currentNode->coordY;
            }
        }
        qDebug()<<" end looking at neighbors";
    }

    qDebug()<<" end while";

    //if no path is found
    if (unvisitedNodes.empty()
        && (currentNode->coordX != goalNode->coordX
        ||  currentNode->coordY != goalNode->coordY))
    {
        noPath = true;
    }
    qDebug()<<" end function";
}

NODE* Djikstra::searchCorrespondingNode(int searchCoordX, int searchCoordY)
{
    std::vector<NODE>::iterator listIndex;
    qDebug()<<"entering "<<"search x : "<< searchCoordX<<"search y : "<<searchCoordY;
    for(listIndex = allNodes.begin();
        listIndex< allNodes.end(); listIndex++)
    {
        int index = std::distance( allNodes.begin(), listIndex);

        int coordX = allNodes.at(index).coordX;
        int coordY = allNodes.at(index).coordY;

        if(coordX == searchCoordX && coordY == searchCoordY)
        {                
            return &allNodes.at(index);
        }
    }
    qDebug()<<"exit failure";
    return NULL;
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
                        std::pair <int,int> newNeighbor (k,l);
                        allNodes.at(index).neighborsList.push_back(newNeighbor);
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
        //qDebug()<<"column : "<<column;

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
                /*
                qDebug()<<"row : "<<rowN;
                qDebug()<<"m : "<<m<<" n : "<<n;
                qDebug()<<"x : "<<newNode.coordX<<" y : "<<newNode.coordY;

                */
                allNodes.push_back(newNode);
            }
            nodeState = NORMAL; //reinitialize

        }
        n = 0;
        configurationSpace.push_back(row);
    }
}

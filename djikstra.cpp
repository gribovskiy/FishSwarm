//Autor : Laila El Hamamsy
//Date Created : Sun October 16th 2016
//Version : 1
//Last Modified :

#include "djikstra.h"

struct NEIGHBOOR{
    NODE *neighboor;
    float distNeighboor;
    NEIGHBOOR (NODE *neighboor, float distNeighboor): neighboor(NULL), distNeighboor(INF){};
};

struct NODE{
    NODE    *parentNode = NULL;
    float   distanceToStart = INF;
    bool    visited = false;
    int     coordX;
    int     coordY;
    NODE    (int coordX, int coordY) : coordX(INF), coordY(INF){};
    std::vector<NEIGHBOOR> neighborsList;
};

Djikstra::Djikstra(int startCoord[2], int goalCoord[2], std::vector< std::vector<int> > newConfigurationSpace, int distNodes)
{
    if (distNodes != 1 && distNodes % 2 == 1)
    {
        distNodes--;
    }

    setConfigurationSpace(newConfigurationSpace, distNodes);
    getGraphFromNodeList();

    startNode->distanceToStart = 0;
    startNode->coordX=startCoord[0]/distNodes;
    startNode->coordY=startCoord[1]/distNodes;

    goalNode->coordX=startCoord[0]/distNodes;
    goalNode->coordY=startCoord[1]/distNodes;


    startNode = searchCorrespondingNode (startNode->coordX, startNode->coordY);
    goalNode  = searchCorrespondingNode (goalNode->coordX, goalNode->coordY);
    ///is it better to do the other initializations for
    ///the other NODES here rather than in the struct itself?

    getGraphFromNodeList();
}

void Djikstra::initialize()
{



}


NODE* Djikstra::searchCorrespondingNode(int searchCoordX, int searchCoordY)
{
    std::vector<NODE>::iterator listIndex;

    for(listIndex = unvisitedNodes->begin();
        listIndex!= unvisitedNodes->end(); listIndex++)
    {
        int index = std::distance( (*unvisitedNodes).begin(), listIndex);

        if(unvisitedNodes->at(index).coordX == searchCoordX
        && unvisitedNodes->at(index).coordY == searchCoordY)

        return &(unvisitedNodes->at(index));
    }
    return NULL;
}


void Djikstra::getGraphFromNodeList()
{
    std::vector<std::vector<int>>:: iterator columnIterator, i;
    std::vector<int>::iterator rowIterator, j;

    std::vector<NODE>::iterator listIndex;

    NODE newNode(INF,INF);
    int m = 0, n = 0;
    int k = 0, l = 0;

/*
    for( columnIterator = configurationSpace->begin() ;
         columnIterator!= configurationSpace->end(); columnIterator++, m++)
    {
        for( rowIterator = (*columnIterator).begin();
             rowIterator!= (*columnIterator).end(); rowIterator++, n++)
        {
            if(configurationSpace->at(m)[n] != FORBIDDEN)
            {
                //identify the node


                for (i = columnIterator -1 ; i <columnIterator+1 ; i++)
                {
                    for (j = rowIterator -1 ; j <rowIterator+1; j++)
                    {
                        if (i != columnIterator && j != rowIterator //to avoid adding the node as it's neighboor
                        && i>=configurationSpace->begin() && i<configurationSpace->end() //to avoid accessing a point outside the configuration space
                        && j>=(*columnIterator).begin() && j<(*columnIterator).end() ) //same
                        {
                            if( *j != FORBIDDEN)
                            {
                                //identify the neighbors

                                //add the neighboors to the neighboorList with associated distance
                            }
                        }
                    }
                }
            }
        }
    }
    */

    //

    for(listIndex = unvisitedNodes->begin();
        listIndex!= unvisitedNodes->end(); listIndex++)
    {
        int index = std::distance( (*unvisitedNodes).begin(), listIndex);
        m = unvisitedNodes->at(index).coordX;
        n = unvisitedNodes->at(index).coordY;

        for (k = m -1 ; k <m+1 ; k++)
        {
            for (l = n -1 ; l < n+1; l++)
            {
                if (k != m && l != n //to avoid adding the node as it's neighboor
                && k >= 0 && k< std::distance(configurationSpace->end(),configurationSpace->begin()) //to avoid accessing a point outside the configuration space
                && l >= 0 && l< std::distance(configurationSpace->end()->end(),configurationSpace->begin()->begin())) //same
                {
                    if( configurationSpace->at(k).at(l) != FORBIDDEN)
                    {
                        NODE *newNode = searchCorrespondingNode(m, n);
                        //add the neighboors to the neighboorList with associated distance
                        if (newNode)
                        {
                            float distNeighbor = sqrt((m-k)*(m-k) +(n-l)*(n-l));
                            NEIGHBOOR newNeighbor(newNode, distNeighbor);
                            unvisitedNodes->at(index).neighborsList.push_back(newNeighbor);
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

    int step = 0, m =0, n =0;
    int nodeState = NORMAL;

    if (distNodes == 1)
    {
         (*configurationSpace) =  newConfigurationSpace;
    }

    step = distNodes/2+1;

    for( columnIterator = newConfigurationSpace.begin() + step;
         columnIterator + step!= newConfigurationSpace.end();
         columnIterator + distNodes, m++)
    {
        for( rowIterator = (*columnIterator).begin()+step;
             rowIterator + step!= (*columnIterator).end();
             rowIterator + distNodes, n++)
        {
            //test the cells around de i, j to determine state : normal, gostraight or forbidden
            for (k = columnIterator-step ; k<k+step;k++)
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

            configurationSpace->at(m).at(n) = nodeState;
            qDebug()<<"m : "<<m<<" n : "<<n<<"state : "<<nodeState;

            if (nodeState != FORBIDDEN)
            {
                unvisitedNodes->push_back(NODE(m,n));
            }
            nodeState = NORMAL; //reinitialize
        }
    }
}

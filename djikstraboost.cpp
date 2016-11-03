#include "djikstraboost.h"
#include <iostream>
#include <math.h>

DjikstraBoost::DjikstraBoost(int newDistNodes, std::vector< std::vector<int> > newConfigurationSpace)
{
    startCell.first  = 0;
    startCell.second = 0;
    goalCell.first   = 0;
    goalCell.second  = 0;

    setNewConfigurationSpace(newConfigurationSpace, newDistNodes);
    computeGraph();
}

DjikstraBoost::DjikstraBoost(int startCoord[2], int goalCoord[2], int newDistNodes,
                             std::vector< std::vector<int> > newConfigurationSpace)
{ 
    setNewConfigurationSpace(newConfigurationSpace, newDistNodes);
    computeGraph();
    computeDjikstraShortestPathAlgorithm(startCoord, goalCoord);
}


void DjikstraBoost::computeGraph(){
    allVertices.clear();
    num_nodes = 0;
    shortestPath.clear();
    pathCoord.clear();
    configurationSpaceToVertexList();
    vertexListToEdgeList();
}

void DjikstraBoost::computeDjikstraShortestPathAlgorithm(int startCoord[2], int goalCoord[2])
{    
    if(!configurationSpace.empty())
    {
        //if the start or goal have been initialized (ie either coord different than 0)
        if (startCell.first||startCell.second||goalCell.first||goalCell.second)
        {
        initializeStartAndGoal(startCoord,goalCoord);
        searchForShortestPath();
        reconstructPath();
        }
    }
}

void DjikstraBoost::initializeStartAndGoal(int startCoord[2], int goalCoord[2]){

    startCell.first  = startCoord[0]/distNodes;
    startCell.second = startCoord[1]/distNodes;
    goalCell.first   =  goalCoord[0]/distNodes;
    goalCell.second  =  goalCoord[1]/distNodes;

    QPoint startPoint(startCell.first,startCell.second),
            goalPoint(goalCell.first,goalCell.second);

    VertexKey startKey(startPoint), goalKey(goalPoint);


    //add start and goal to vertice list if they are not already there
    if(!allVertices.value(startKey))
    {
        addVertex(startCell.first, startCell.second);
        //ADD CORRESPONDING EDGES
    }
    if(!allVertices.value(goalKey))
    {
        addVertex(goalCell.first, goalCell.second);
        //ADD CORREPONDING EDGES
    }

    startVertex = allVertices.value(startKey);
    goalVertex = allVertices.value(goalKey);
}

void DjikstraBoost::reconstructPath()
{
    std::cout << "Path from ("<< startCell.first<<" ; "<<startCell.second<<")"
              << " to ("<< goalCell.first<<" ; "<<goalCell.second<<")"<< std::endl;

    std::vector<boost::graph_traits<UndirectedGraph>::vertex_descriptor >::reverse_iterator it;

    for (it = shortestPath.rbegin(); it != shortestPath.rend(); ++it)
    {
        int coordX = myGraph[*it].pos.first;
        int coordY = myGraph[*it].pos.second;
        int index = std::distance(shortestPath.rbegin(),it);
        std::cout << *it << " ";
        std::cout << "("<<coordX<<", "<<coordY<<")";
        pathCoord.push_back(std::make_pair(coordX*distNodes, coordY*distNodes));
    }
    std::cout << std::endl;
}

std::vector<std::pair <int,int>> DjikstraBoost::getPath()
{
    return pathCoord;
}

void DjikstraBoost::searchForShortestPath()
{
    // Create property_map from edges to weights
    boost::property_map<UndirectedGraph, boost::edge_weight_t>::type weightmap = get(boost::edge_weight, myGraph);

    // Create vectors to store the predecessors (p) and the distances from the root (d)
    std::vector<Vertex> predecessors(num_vertices(myGraph));
    std::vector<int> distances(num_vertices(myGraph));

    // Evaluate Dijkstra on graph g with source s, predecessor_map p and distance_map d
    boost::dijkstra_shortest_paths(myGraph, startVertex,
                                   boost::predecessor_map(&predecessors[0]).distance_map(&distances[0]));

    //p[] is the predecessor map obtained through dijkstra
    //name[] is a vector with the names of the vertices
    //s and goal are vertex descriptors

    boost::graph_traits<UndirectedGraph>::vertex_descriptor currentVertex = goalVertex;

    while(currentVertex!=startVertex)
    {
        shortestPath.push_back(currentVertex);
        currentVertex = predecessors[currentVertex];
    }
    shortestPath.push_back(startVertex);
}

void DjikstraBoost::configurationSpaceToVertexList()
{
    int i, j;

    for (i = 0 ; i<width ; i++)
    {
        for (j = 0 ; j<width ; j++)
        {
            if (configurationSpace.at(i).at(j)!= OCCUPIED)
            {
                addVertex(i,j);
            }
        }
    }
}

void DjikstraBoost::vertexListToEdgeList()
{
    int i;
    //Function : Vertex List -> Edge List
    for (i = 0 ; i<num_nodes ; i++)
    {
        //take each vertex
        int currentX = myGraph[i].pos.first;
        int currentY = myGraph[i].pos.second;

        QPoint    currentPoint(currentX,currentY);
        VertexKey currentKey(currentPoint);
        Vertex    currentVertex = allVertices.value(currentKey);

        //look at the cells around it
        vertexNeighborsAndEdges(currentX, currentY, currentVertex);
    }
}

void DjikstraBoost::vertexNeighborsAndEdges(int currentX, int currentY, Vertex currentVertex)
{
    int k,l;

    for (k = currentX-1; k<=currentX+1 ;  k++)
    {
        for (l = currentY-1; l<=currentY+1; l++)
        {
            if(k>=0 && l>=0 && l<height && k<width &&!(k==currentX && l==currentY)) //to stay in bounds
            {
                if (configurationSpace.at(k).at(l)!=OCCUPIED)
                {
                    addEdge(currentVertex, currentX, currentY, k,l);
                }
            }
        }
    }
}

void DjikstraBoost::addEdge(Vertex currentVertex, int currentX, int currentY, int adjacentX, int adjacentY)
{
    if(adjacentX>=0 && adjacentY>=0
            && adjacentY<width && adjacentX<height
            &&!(adjacentX==currentX && adjacentY==currentY)) //to stay in bounds and avoid current cell
    {
        float distance = sqrt(pow(currentX-adjacentX, 2)
                              + pow(adjacentY-currentY, 2));
        QPoint adjacentPoint(adjacentX,adjacentY);
        VertexKey adjacentKey(adjacentPoint);
        Vertex adjacentVertex = allVertices.value(adjacentKey);
        boost::add_edge(currentVertex, adjacentVertex, distance, myGraph);
    }
}

void DjikstraBoost::setNewConfigurationSpace(std::vector< std::vector<int> > newConfigurationSpace, int newDistNodes)
{
    std::vector<std::vector<int>>:: iterator columnIterator;
    std::vector<int>::iterator rowIterator;

    distNodes = newDistNodes;

    if (distNodes == 0)
    {
        return;
    }

    if (distNodes % 2 == 0)
    {
        distNodes--;
    }

    myGraph.clear();

    int nodeState = FREE, step = distNodes/2;
    width = 0;
    height = 0;

    newConfigurationSpace.at(0).at(0);

    std::vector<int> row;
    int col, lin;
    int configSpaceWidth  = newConfigurationSpace.size();
    int configSpaceHeight = newConfigurationSpace.at(0).size();

    for (col = step ; (col+distNodes+step+1)<configSpaceWidth; col+=distNodes)
    {
        row.clear();
        height = 0;

        for (lin = step ; (lin+distNodes+step+1) < configSpaceHeight ; lin+=distNodes)
        {
            int k, l;
            int nodeState = getNodeState(newConfigurationSpace, col, lin, step);
            row.push_back(nodeState);
            height++;
        }
        configurationSpace.push_back(row);
        width++;
    }
}

void DjikstraBoost::addVertex(int x, int y){
    Vertex vertex = boost::add_vertex(myGraph);
    myGraph[num_nodes].pos = std::make_pair(x,y);
    QPoint point(x,y);
    VertexKey vKey(point);
    allVertices.insert(vKey,vertex);
    num_nodes++;
}

int DjikstraBoost::getNodeState(std::vector< std::vector<int> > newConfigurationSpace,
                                int column,int row,int step){
    //test the surronding cells to determine state : Free, Hallway or Occupied

    int k, l;

    int nodeState = FREE;

    for (k = column-step ; k<=column+step; k++)
    {
        for (l = row-step; l<=row+step ; l++)
        {
            switch (newConfigurationSpace.at(k).at(l))
            {
            case FREE : //because initialized NORMAL
                break;

            case HALLWAY :
                if (nodeState == FREE)
                {
                    nodeState = HALLWAY;
                }
                break;

            case OCCUPIED :
                nodeState = OCCUPIED;
                break;

            default :
                break;
            }
        }
    }
    return nodeState;
}

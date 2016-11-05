#include "djikstraboost.h"
#include <iostream>
#include <math.h>

DjikstraBoost::DjikstraBoost(int newDistNodes, std::vector< std::vector<int> > newConfigurationSpace)
{ 
    //Make sure the distance between the nodes /vertex are appropriate
    distNodes = newDistNodes;
    if (distNodes == 0)
    {
        return;
    }

    if (distNodes % 2 == 0)
    {
        distNodes--;
    }

    setNewConfigurationSpace(newConfigurationSpace);
}

std::vector<std::pair <int,int>> DjikstraBoost::getPath(QPoint startCoord, QPoint goalCoord)
{
    myGraph.clear();
    computeDjikstraShortestPathAlgorithm(startCoord, goalCoord);
    return pathCoord;
}

void DjikstraBoost::computeDjikstraShortestPathAlgorithm(QPoint startCoord, QPoint goalCoord)
{
    //reinitialize the vertex list, number of nodes, shortest path and the path coordinates
    allVertices.clear();
    num_nodes = 0;
    shortestPath.clear();
    pathCoord.clear();

    //Configuration space -> vertex list -> edge list -> shortest path
    configurationSpaceToVertexList();
    initializeStartAndGoal(startCoord,goalCoord);
    vertexListToEdgeList();
    searchForShortestPath();
    reconstructPath();
}

void DjikstraBoost::initializeStartAndGoal(QPoint startCoord, QPoint goalCoord)
{
    startCell.first  = startCoord.x()/distNodes;
    startCell.second = startCoord.y()/distNodes;
    goalCell.first   =  goalCoord.x()/distNodes;
    goalCell.second  =  goalCoord.y()/distNodes;

    QPoint startPoint = startCoord/distNodes,
            goalPoint = goalCoord/distNodes;

    VertexKey startKey(startPoint), goalKey(goalPoint);

    //add start and goal to vertice list if they are not already there
    if(!allVertices.value(startKey))
    {
        addNewVertex(startCell.first, startCell.second);
    }
    if(!allVertices.value(goalKey))
    {
        addNewVertex(goalCell.first, goalCell.second);
    }

    startVertex = allVertices.value(startKey);
    goalVertex = allVertices.value(goalKey);
}

void DjikstraBoost::reconstructPath()
{
    std::cout << "Path from ("<< startCell.first<<" ; "<<startCell.second<<")"
              << " to ("<< goalCell.first<<" ; "<<goalCell.second<<")"<< std::endl;

    std::vector<boost::graph_traits<UndirectedGraph>::vertex_descriptor >::reverse_iterator it;

    //print out the shortest path
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


void DjikstraBoost::searchForShortestPath() //highly based on the djikstra boost example
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

    //reconstruct the shortest path based on the parent list
    while(currentVertex!=startVertex)
    {
        shortestPath.push_back(currentVertex);
        currentVertex = predecessors[currentVertex];
    }
    shortestPath.push_back(startVertex);
}

void DjikstraBoost::configurationSpaceToVertexList()
{
    //get all the accesible vertices in our configuration space
    int i, j;

    for (i = 0 ; i<width ; i++)
    {
        for (j = 0 ; j<width ; j++)
        {
            if (configurationSpace.at(i).at(j)!= OCCUPIED)
            {
                addNewVertex(i,j);
            }
        }
    }
}

void DjikstraBoost::vertexListToEdgeList() //compute all the edges
{
    int i, k, l;

    for (i = 0 ; i<num_nodes ; i++)
    {
        //take each vertex
        int currentX = myGraph[i].pos.first;
        int currentY = myGraph[i].pos.second;

        QPoint    currentPoint(currentX,currentY);
        VertexKey currentKey(currentPoint);
        Vertex    currentVertex = allVertices.value(currentKey);

        //look at the cells around it
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
}

void DjikstraBoost::addEdge(Vertex currentVertex, int currentX, int currentY, int adjacentX, int adjacentY)
{
    if(adjacentX>=0 && adjacentY>=0
            && adjacentY<width && adjacentX<height
            &&!(adjacentX==currentX && adjacentY==currentY)) //to stay in bounds and avoid current cell
    {
        //calculate the distance between the 2 points
        float distance = sqrt(pow(currentX-adjacentX, 2)
                              + pow(adjacentY-currentY, 2));
        // add the edge between the 2 vertices
        QPoint adjacentPoint(adjacentX,adjacentY);
        VertexKey adjacentKey(adjacentPoint);
        Vertex adjacentVertex = allVertices.value(adjacentKey);
        boost::add_edge(currentVertex, adjacentVertex, distance, myGraph);
    }
}

void DjikstraBoost::setNewConfigurationSpace(std::vector< std::vector<int> > newConfigurationSpace)
{
    std::vector<int> row;
    int col, lin;
    int configSpaceWidth  = newConfigurationSpace.size();
    int configSpaceHeight = newConfigurationSpace.at(0).size();
    int nodeState = FREE, step = distNodes/2;

    //Reset configuration space width and height parameters
    width = 0;
    height = 0;

    //Iterate through the center of the new cells separated by distNodes and after
    //determinating the cell state add them to the new configuration space
    for (col = step ; (col+distNodes+step+1)<configSpaceWidth; col+=distNodes)
    {
        row.clear();
        height = 0;

        for (lin = step ; (lin+distNodes+step+1) < configSpaceHeight ; lin+=distNodes)
        {
            int nodeState = getNodeState(newConfigurationSpace, col, lin, step);
            row.push_back(nodeState);
            height++;
        }
        configurationSpace.push_back(row);
        width++;
    }
}

void DjikstraBoost::addNewVertex(int x, int y)
{
    Vertex vertex = boost::add_vertex(myGraph);
    myGraph[num_nodes].pos = std::make_pair(x,y);
    QPoint point(x,y);
    VertexKey vKey(point);
    allVertices.insert(vKey,vertex);
    num_nodes++;
}

int DjikstraBoost::getNodeState(std::vector< std::vector<int> > newConfigurationSpace,
                                int column,int row,int step)
{
    //test the surronding cell to determine the state : Free, Hallway or Occupied
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

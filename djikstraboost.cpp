//Autor : Laila El Hamamsy
//Date Created : October 2016
//Version :
//Last Modified :

#include "djikstraboost.h"
#include <iostream>
#include <math.h>

// TODO : to add comments explaining what this method does, you can copy them from the .h file
/*!
 * Constructor. It receives the distance between nodes and the configuration
 * space as parameters.
 */
DjikstraBoost::DjikstraBoost(int newDistNodes, std::vector<std::vector<State>> newConfigurationSpace)
{
    //Make sure the distance between the nodes /vertex are appropriate
    m_distNodes = newDistNodes;
    if (m_distNodes == 0)
    {
        return;
    }

    if (m_distNodes % 2 == 0)
    {
        m_distNodes--;
    }

    setNewConfigurationSpace(newConfigurationSpace);
}

std::vector<QPoint> DjikstraBoost::getPath(QPoint startCoord, QPoint goalCoord)
{
    m_myGraph.clear();
    computeDjikstraShortestPathAlgorithm(startCoord, goalCoord);
    return m_pathCoord;
}

// FIXME : at the moment the graph is redone every time you do the path planning, why?
void DjikstraBoost::computeDjikstraShortestPathAlgorithm(QPoint startCoord, QPoint goalCoord)
{
    //reinitialize the vertex list, number of nodes, shortest path and the path coordinates
    m_allVertices.clear();
    m_num_nodes = 0;
    m_shortestPath.clear();
    m_pathCoord.clear();

    //Configuration space -> vertex list -> edge list -> shortest path
    configurationSpaceToVertexList();
    initializeStartAndGoal(startCoord,goalCoord);
    vertexListToEdgeList();
    searchForShortestPath();
    reconstructPath();
}

void DjikstraBoost::initializeStartAndGoal(QPoint startCoord, QPoint goalCoord)
{
    m_startCell.setX(startCoord.x()/m_distNodes);
    m_startCell.setY(startCoord.y()/m_distNodes);
    m_goalCell .setX(goalCoord. x()/m_distNodes);
    m_goalCell .setY(goalCoord. y()/m_distNodes);

    QPoint startPoint = startCoord/m_distNodes,
            goalPoint = goalCoord/m_distNodes;

    VertexKey startKey(startPoint), goalKey(goalPoint);

    //add start and goal to vertice list if they are not already there
    if(!m_allVertices.value(startKey))
    {
        addNewVertex(m_startCell.x(), m_startCell.y());
    }
    if(!m_allVertices.value(goalKey))
    {
        addNewVertex(m_goalCell.x(), m_goalCell.y());
    }

    m_startVertex = m_allVertices.value(startKey);
    m_goalVertex = m_allVertices.value(goalKey);
}

void DjikstraBoost::reconstructPath()
{
    std::cout << "Path from ("<< m_startCell.x()<<" ; "<<m_startCell.y()<<")"
              << " to ("<< m_goalCell.x()<<" ; "<<m_goalCell.y()<<")"<< std::endl;

    std::vector<boost::graph_traits<UndirectedGraph>::vertex_descriptor >::reverse_iterator it;

    QPoint point;
    //otherwise print out the shortest path
    for (it = m_shortestPath.rbegin(); it != m_shortestPath.rend(); ++it)
    {
        point.setX(m_myGraph[*it].pos.first);
        point.setY(m_myGraph[*it].pos.second);
        std::cout << *it << " ";
        std::cout << "("<<point.x()<<", "<<point.y()<<")";
        m_pathCoord.push_back(point*m_distNodes);
    }
    std::cout << std::endl;

    //if there is no path to the goal
    if (point.x() != m_goalCell.x() && point.y()!= m_goalCell.y())
    {
        m_pathCoord.clear();
        return;
    }
}


void DjikstraBoost::searchForShortestPath() //highly based on the djikstra boost example
{
    // Create property_map from edges to weights
    boost::property_map<UndirectedGraph, boost::edge_weight_t>::type weightmap = get(boost::edge_weight, m_myGraph);

    // Create vectors to store the predecessors (p) and the distances from the root (d)
    std::vector<Vertex> predecessors(num_vertices(m_myGraph));
    std::vector<float> distances(num_vertices(m_myGraph));

    // Evaluate Dijkstra on graph g with source s, predecessor_map p and distance_map d
    boost::dijkstra_shortest_paths(m_myGraph, m_startVertex,
                                   boost::predecessor_map(&predecessors[0]).distance_map(&distances[0]));

    //p[] is the predecessor map obtained through dijkstra
    //name[] is a vector with the names of the vertices
    //s and goal are vertex descriptors

    boost::graph_traits<UndirectedGraph>::vertex_descriptor currentVertex = m_goalVertex;

    //reconstruct the shortest path based on the parent list
    while(currentVertex!=m_startVertex)
    {
        m_shortestPath.push_back(currentVertex);
        currentVertex = predecessors[currentVertex];
    }
    m_shortestPath.push_back(m_startVertex);
}

void DjikstraBoost::configurationSpaceToVertexList()
{
    //get all the accesible vertices in our configuration space
    int i, j;

    for (i = 0 ; i<m_width ; i++)
    {
        for (j = 0 ; j<m_width ; j++)
        {
            if (m_configurationSpace.at(i).at(j)!= State::OCCUPIED)
            {
                addNewVertex(i,j);
            }
        }
    }
}

void DjikstraBoost::vertexListToEdgeList() //compute all the edges
{
    int i, k, l;

    for (i = 0 ; i<m_num_nodes ; i++)
    {
        //take each vertex
        int currentX = m_myGraph[i].pos.first;
        int currentY = m_myGraph[i].pos.second;

        QPoint    currentPoint(currentX,currentY);
        VertexKey currentKey(currentPoint);
        Vertex    currentVertex = m_allVertices.value(currentKey);

        //look at the cells around it
        for (k = currentX-1; k<=currentX+1 ;  k++)
        {
            for (l = currentY-1; l<=currentY+1; l++)
            {
                if(k>=0 && l>=0 && l<m_height && k<m_width &&!(k==currentX && l==currentY)) //to stay in bounds
                {
                    if (m_configurationSpace.at(k).at(l)!=State::OCCUPIED)
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
            && adjacentY<m_width && adjacentX<m_height
            &&!(adjacentX==currentX && adjacentY==currentY)) //to stay in bounds and avoid current cell
    {
        //calculate the distance between the 2 points
        float distance = sqrt(pow(currentX-adjacentX, 2)
                              + pow(adjacentY-currentY, 2));
        // add the edge between the 2 vertices
        QPoint adjacentPoint(adjacentX,adjacentY);
        VertexKey adjacentKey(adjacentPoint);
        Vertex adjacentVertex = m_allVertices.value(adjacentKey);
        boost::add_edge(currentVertex, adjacentVertex, distance, m_myGraph);
    }
}

void DjikstraBoost::setNewConfigurationSpace(std::vector<std::vector<State>> newConfigurationSpace)
{
    std::vector<State> row;
    int col, lin;
    int step = m_distNodes/2;
    State nodeState;

    int configSpaceWidth  = newConfigurationSpace.size();
    int configSpaceHeight;

    if (!newConfigurationSpace.at(0).empty())
    {
        configSpaceHeight = newConfigurationSpace.at(0).size();
    }
    else configSpaceHeight = 0;


    //Reset configuration space width and height parameters
    m_width = 0;
    m_height = 0;

    //Iterate through the center of the new cells separated by distNodes and after
    //determinating the cell state add them to the new configuration space

    for (col = step ; col<configSpaceWidth; col+=m_distNodes)
    {
        row.clear();
        m_height = 0;

        for (lin = step ; lin< configSpaceHeight ; lin+=m_distNodes)
        {
            nodeState = getNodeState(newConfigurationSpace, col, lin, step);
            row.push_back(nodeState);
            m_height++;
        }
        m_configurationSpace.push_back(row);
        m_width++;
    }
}

void DjikstraBoost::addNewVertex(int x, int y)
{
    Vertex vertex = boost::add_vertex(m_myGraph);
    m_myGraph[m_num_nodes].pos = std::make_pair(x,y);
    QPoint point(x,y);
    VertexKey vKey(point);
    m_allVertices.insert(vKey,vertex);
    m_num_nodes++;
}

State DjikstraBoost::getNodeState(std::vector<std::vector<State>> newConfigurationSpace,
                                int column,int row,int step)
{
    //test the surronding cell to determine the state : FREE, HALLWAY or OCCUPIED
    int k, l;
    int configSpaceWidth  = newConfigurationSpace.size();
    int configSpaceHeight;

    if (!newConfigurationSpace.at(0).empty())
    {
        configSpaceHeight = newConfigurationSpace.at(0).size();
    }
    else configSpaceHeight = 0;

    //case FREE if one cell FREE
    //int nodeState = OCCUPIED;

    //case OCCUPIED if one cell OCCUPIED
    State nodeState = State::FREE;

    for (k = column-step ; k<=column+step; k++)
    {
        for (l = row-step; l<=row+step ; l++)
        {
            //if we are in bounds
            if (k>=0 && l>=0 && k<configSpaceWidth && l<configSpaceHeight)
            {
                //Case : OCCUPIED if one cell is OCCUPIED (OCCUPIED > HALLWAY > FREE)

                switch (newConfigurationSpace.at(k).at(l))
                {
                case State::FREE : //because initialized FREE
                    break;

                case State::HALLWAY :
                    if (nodeState == State::FREE)
                    {
                        nodeState = State::HALLWAY;
                    }
                    break;

                case State::OCCUPIED :
                    nodeState = State::OCCUPIED;
                    break;

                default :
                    break;
                }
            }
        }
    }
    return nodeState;
}

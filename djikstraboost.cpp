//Autor : Laila El Hamamsy
//Date Created : October 2016
//Version : 5
//Last Modified : 26.12.2016

#include "djikstraboost.h"
#include <iostream>
#include <math.h>


//----------------------------------------------------------------------------//
//-------------------------------Class Constructors---------------------------//
//----------------------------------------------------------------------------//

/*!
 * Class Constructor. It receives the distance between nodes and the configuration
 * space as parameters.
 */
DjikstraBoost::DjikstraBoost(int newDistNodes, std::vector<std::vector<State>> newConfigurationSpace)
{
    //! Make sure the distance between the nodes /vertex are appropriate
    m_distNodes = newDistNodes;
    //! if the distance is equal to 0, return
    if (m_distNodes == 0)
    {
        return;
    }

    //! if the distance is even, decrement so the node will be at the center of the cell
    if (m_distNodes % 2 == 0)
    {
        m_distNodes--;
    }

    //! set up the new configuration space.
    setNewConfigurationSpace(newConfigurationSpace);
}


//----------------------------------------------------------------------------//
//-------------------------------Exported Members-----------------------------//
//----------------------------------------------------------------------------//


/*!
 * Exported Member. It receives the coordinates of the strat and goal position
 * as parameters. It returns the vector of points making up the dijstra shortest
 * path
 */
std::vector<QPoint> DjikstraBoost::getDijkstraPath(QPoint startCoord, QPoint goalCoord)
{
    //! clear the graph
    m_myGraph.clear();

    //! compute the shortest path from the start to the goal
    computeDjikstraShortestPathAlgorithm(startCoord, goalCoord);

    //! synthetize the path down to the essential points
    std::vector<QPoint> path  = getReducedPath();

    return path;
}



//----------------------------------------------------------------------------//
//----------------------------Non Exported Members----------------------------//
//----------------------------------------------------------------------------//


/*!
 * Non Exported Member. this method identifies the essential nodes in the path
 * and returns the reduced path.
 */

std::vector<QPoint> DjikstraBoost::getReducedPath()
{
    int prevDx, currentDx, prevDy, currentDy;
    std::vector<QPoint> path;

    //! if the computed dijkstra path is not empty
    if (!m_pathCoord.empty())
    {
        //! add the starting point
        path.push_back(m_pathCoord.at(0));
        //! if there are more than 2 nodes in the path
        if(m_pathCoord.size()>2)
        {
            //! compute the fisrt difference of position along x and y
            prevDx = m_pathCoord.at(1).x()-m_pathCoord.at(0).x();
            prevDy = m_pathCoord.at(1).y()-m_pathCoord.at(0).y();

            //! for all the other points in the path
            for(int i = 2 ; i<(int)m_pathCoord.size();i++)
            {
                //! compute the curent difference of position along x and y
                currentDx = m_pathCoord.at(i).x()-m_pathCoord.at(i-1).x();
                currentDy = m_pathCoord.at(i).y()-m_pathCoord.at(i-1).y();

                //! if the slope is different
                if(!(prevDx == currentDx && prevDy == currentDy))
                {
                    //! add the new point to the path
                    path.push_back(m_pathCoord.at(i));
                }
                //! update the current differences of positions along x and y
                prevDx = currentDx;
                prevDy = currentDy;
            }
        }
    }
    //! return the reduced path
    return path;
}

/*!
 * Non Exported Member. this method computes djikstra's shortest path for a
 * given configuration space. It receives the start and goal points as parameters
 */

// FIXME : at the moment the graph is redone every time you do the path planning, why?
void DjikstraBoost::computeDjikstraShortestPathAlgorithm(QPoint startCoord, QPoint goalCoord)
{
    //! reinitialize the vertex list, number of nodes, shortest path and the path coordinates
    m_allVertices.clear();
    m_num_nodes = 0;
    m_shortestPath.clear();
    m_pathCoord.clear();

    //! Configuration space -> vertex list -> edge list -> shortest path
    //! get the new vertex list
    configurationSpaceToVertexList();
    //! intializae and add start and goal to the vertex list if they are not already there
    initializeStartAndGoal(startCoord,goalCoord);
    //! get the new edge list
    vertexListToEdgeList();
    //! compute the shortest path with dijkstra
    searchForShortestPath();
    //! reconstruct the path using the predecessor list
    reconstructPath();
}

/*!
 * Non Exported Member. This method initializes the start and goal nodes
 * It receives the start and goal points as parameters
 */
void DjikstraBoost::initializeStartAndGoal(QPoint startCoord, QPoint goalCoord)
{

    //! initialize the start and goal coordinates in the discretized configuration space
    m_startCell = startCoord/m_distNodes,
    m_goalCell = goalCoord/m_distNodes;

    //! create the corresponding keys
    VertexKey startKey(m_startCell), goalKey(m_goalCell);

    //! add start and goal to vertice list if they are not already there
    if(!m_allVertices.value(startKey))
    {
        addNewVertex(m_startCell.x(), m_startCell.y());
    }
    if(!m_allVertices.value(goalKey))
    {
        addNewVertex(m_goalCell.x(), m_goalCell.y());
    }

    //! intialize start and goal vertex
    m_startVertex = m_allVertices.value(startKey);
    m_goalVertex = m_allVertices.value(goalKey);
}

/*!
 * Non Exported Member. this method reconstructs the path using the boost
 * predecessor list
 */
void DjikstraBoost::reconstructPath()
{
    std::cout << "Path from ("<< m_startCell.x()<<" ; "<<m_startCell.y()<<")"
              << " to ("<< m_goalCell.x()<<" ; "<<m_goalCell.y()<<")"<< std::endl;

    std::vector<boost::graph_traits<UndirectedGraph>::vertex_descriptor >::reverse_iterator it;

    QPoint point;

    //! print out the shortest path by iterating from the end to the start
    for (it = m_shortestPath.rbegin(); it != m_shortestPath.rend(); ++it)
    {
        //! identify the point in the graph
        point.setX(m_myGraph[*it].pos.first);
        point.setY(m_myGraph[*it].pos.second);
        std::cout << *it << " ";
        std::cout << "("<<point.x()<<", "<<point.y()<<")";
        //! add the point to the path
        m_pathCoord.push_back(point*m_distNodes);
    }
    std::cout << std::endl;

    //! if there is no path to the goal
    if (point.x() != m_goalCell.x() && point.y()!= m_goalCell.y())
    {
        //! clear the path
        m_pathCoord.clear();
        return;
    }
}

/*!
 * Non Exported Member. this method searches for the shortest path using
 * boost library
 * ! highly based on the djikstra boost example
 */
void DjikstraBoost::searchForShortestPath()
{
    //! Create property_map from edges to weights
    boost::property_map<UndirectedGraph, boost::edge_weight_t>::type weightmap = get(boost::edge_weight, m_myGraph);

    //! Create vectors to store the predecessors (p) and the distances from the root (d)
    std::vector<Vertex> predecessors(num_vertices(m_myGraph));
    std::vector<float> distances(num_vertices(m_myGraph));

    //! Evaluate Dijkstra on graph g with source s, predecessor_map p and distance_map d
    boost::dijkstra_shortest_paths(m_myGraph, m_startVertex,
                                   boost::predecessor_map(&predecessors[0]).distance_map(&distances[0]));

    //! p[] is the predecessor map obtained through dijkstra
    //! name[] is a vector with the names of the vertices
    //! s and goal are vertex descriptors

    boost::graph_traits<UndirectedGraph>::vertex_descriptor currentVertex = m_goalVertex;

    //! reconstruct the shortest path based on the parent list
    while(currentVertex!=m_startVertex)
    {
        m_shortestPath.push_back(currentVertex);
        currentVertex = predecessors[currentVertex];
    }
    m_shortestPath.push_back(m_startVertex);
}

/*!
 * Non Exported Member. This method generates the vertex list from the
 * configuration space
 */
void DjikstraBoost::configurationSpaceToVertexList()
{
    //! add all the accesible vertices in our configuration space
    int i, j;

    //! iterate throught the reduced configuration space
    for (i = 0 ; i<m_width ; i++)
    {
        for (j = 0 ; j<m_width ; j++)
        {
            //! if the considered cell is not occupied
            if (m_configurationSpace.at(i).at(j)!= State::OCCUPIED)
            {
                //! add the coordinates to the vertex list
                addNewVertex(i,j);
            }
        }
    }
}

/*!
 * Non Exported Member. this method generates the edge list from the vertex list
 */
void DjikstraBoost::vertexListToEdgeList() //!compute all the edges
{
    int i, k, l;

    //! for all the nodes / vertices in the graph
    for (i = 0 ; i<m_num_nodes ; i++)
    {
        //! take each vertex
        int currentX = m_myGraph[i].pos.first;
        int currentY = m_myGraph[i].pos.second;

        //! generate its key
        QPoint    currentPoint(currentX,currentY);
        VertexKey currentKey(currentPoint);
        //! get the corresponding vertex from the vertex list
        Vertex    currentVertex = m_allVertices.value(currentKey);

        //! look at the cells around it
        for (k = currentX-1; k<=currentX+1 ;  k++)
        {
            for (l = currentY-1; l<=currentY+1; l++)
            {
                //! if the cell is in bounds and is not the current cell
                if(k>=0 && l>=0 && l<m_height && k<m_width &&!(k==currentX && l==currentY))
                {
                    //! and if the configuration space is not occupied
                    if (m_configurationSpace.at(k).at(l)!=State::OCCUPIED)
                    {
                        //! add the edge
                        addEdge(currentVertex, currentX, currentY, k,l);
                    }
                }
            }
        }
    }
}


/*!
 * Non Exported Member. this method adds a new edge to the graph
 */
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

/*!
 * Non Exported Member. This method updates the configuration space. It receives
 * the new configuration space as parameters.
 */
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

/*!
 * Non Exported Member. this method adds a new vertex to the graph given
 * it's x, y coordinates.
 */

void DjikstraBoost::addNewVertex(int x, int y)
{
    Vertex vertex = boost::add_vertex(m_myGraph);
    m_myGraph[m_num_nodes].pos = std::make_pair(x,y);
    QPoint point(x,y);
    VertexKey vKey(point);
    m_allVertices.insert(vKey,vertex);
    m_num_nodes++;
}


/*!
 * Non Exported Member. This method identifies whether the node is free,
 * a hallway or occupied. It receives the configuration space as well
 * as the coordinates of the given node (row, column) and the step between
 * the nodes
 */

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

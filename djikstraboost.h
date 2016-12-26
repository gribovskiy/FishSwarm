//Autor : Laila El Hamamsy
//Date Created : October 2016
//Version : 5
//Last Modified : 26.12.2016
//inspiration des tutoriels : http://www.technical-recipes.com/2015/getting-started-with-the-boost-graph-library/#Directed
// + http://stackoverflow.com/questions/24366642/how-do-i-change-the-edge-weight-in-a-graph-using-the-boost-graph-library
// + http://www.boost.org/doc/libs/1_46_1/libs/graph/example/dijkstra-example.cpp
// + http://stackoverflow.com/questions/3100146/adding-custom-vertices-to-a-boost-graph
// + http://www.boost.org/doc/libs/1_46_1/libs/graph/example/dijkstra-example.cpp


#ifndef DJIKSTRABOOST_H
#define DJIKSTRABOOST_H

#include <QWidget>
#include <QMap>
#include <QPoint>
#include <QDebug>

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>

#include "constants.h"

// FIXME : need a comment for every method (and for every class), for instance:

struct vertex_info {
    std::pair<int,int> pos;
};

//Adjacency list for the directed graph
typedef boost::adjacency_list <boost::listS, boost::vecS, boost::undirectedS, vertex_info,
boost::property < boost::edge_weight_t, float >> UndirectedGraph;

typedef boost::graph_traits<UndirectedGraph>::vertex_descriptor Vertex;
typedef std::pair<int, int> Edge;

//http://www.developpez.net/forums/d1298928/c-cpp/bibliotheques/qt/qmap-operator-qpoint/
// FIXME : you don't need a special class for this, it's an overkill
class VertexKey{
public:
    VertexKey(QPoint const & point): p(point){}
    QPoint operator() (){return p;}
    QPoint const & operator()() const{return p;}
    bool operator < (VertexKey const & rhs) const
    {
        return p.x() < rhs.p.x() ||
                ( p.x()== rhs.p.x() && p.y() < rhs.p.y());
    }
    bool operator < (QPoint const & rhs ) const{
        return (*this) < VertexKey(rhs);
    }
    //---------------------------------------------
    int vertexKey_getX(){return p.x();}
    int vertexKey_getY(){return p.y();}

private:
    QPoint p;
};

struct VertexProperties{
    int x;
    int y;
};

class DjikstraBoost
{

public:

    //-------------------------------------//
    //---------Class Constructors----------//
    //-------------------------------------//

    /*!
     * Class Constructor. It receives the distance between nodes and the configuration
     * space as parameters.
     */
    DjikstraBoost(int newDistNodes, std::vector<std::vector<State>> newConfigurationSpace);

    //-------------------------------------//
    //---------Exported Members------------//
    //-------------------------------------//


    /*!
     * Exported Member. It receives the coordinates of the strat and goal position
     * as parameters. It returns the vector of points making up the dijstra shortest
     * path
     */
    std::vector<QPoint> getDijkstraPath(QPoint startCoord, QPoint goalCoord);

private :

    //-------------------------------------//
    //-------Non Exported Members----------//
    //-------------------------------------//

    /*!
     * Non Exported Member. this method identifies the essential nodes in the path
     * and returns the reduced path.
     */
    std::vector<QPoint> getReducedPath();

    /*!
     * Non Exported Member. This method updates the configuration space. It receives
     * the new configuration space as parameters.
     */
    void setNewConfigurationSpace(std::vector<std::vector<State>> newConfigurationSpace);

    /*!
     * Non Exported Member. This method identifies whether the node is free,
     * a hallway or occupied. It receives the configuration space as well
     * as the coordinates of the given node (row, column) and the step between
     * the nodes
     */
    State  getNodeState(std::vector<std::vector<State>> newConfigurationSpace,
                      int column,int row,int step);

    /*!
     * Non Exported Member. This method generates the vertex list from the
     * configuration space
     */
    void configurationSpaceToVertexList();

    /*!
     * Non Exported Member. This method initializes the start and goal nodes
     * It receives the start and goal points as parameters
     */
    void initializeStartAndGoal(QPoint startCoord, QPoint goalCoord);


    /*!
     * Non Exported Member. this method computes djikstra's shortest path for a
     * given configuration space. It receives the start and goal points as parameters
     */
    void computeDjikstraShortestPathAlgorithm(QPoint startCoord, QPoint goalCoord);


    /*!
     * Non Exported Member. this method adds a new vertex to the graph given
     * it's x, y coordinates.
     */
    void addNewVertex(int x, int y);

    /*!
     * Non Exported Member. this method generates the edge list from the vertex list
     */
    void vertexListToEdgeList();

    /*!
     * Non Exported Member. this method adds a new edge to the graph
     */
    void addEdge(Vertex currentVertex, int currentX, int currentY,
                 int adjacentX, int adjacentY);

    /*!
     * Non Exported Member. this method searches for the shortest path using
     * boost library
     */
    void searchForShortestPath();

    /*!
     * Non Exported Member. this method reconstructs the path using the boost
     * predecessor list
     */
    void reconstructPath();

    //! Graph and Map instanciation
    UndirectedGraph             m_myGraph;
    QMap <VertexKey, Vertex>    m_allVertices;  //find the vertex
    QPoint                      m_startCell, m_goalCell;
    Vertex                      m_startVertex, m_goalVertex;
    int                         m_width, m_height, m_num_nodes;
    int                         m_distNodes;
    std::vector<boost::graph_traits<UndirectedGraph>::vertex_descriptor > m_shortestPath;
    std::vector<QPoint> m_pathCoord;
    std::vector<std::vector<State>> m_configurationSpace;
};

#endif // DJIKSTRABOOST_H

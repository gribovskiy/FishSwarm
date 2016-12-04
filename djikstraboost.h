//Autor : Laila El Hamamsy
//Date Created : October 2016
//Version :
//Last Modified :
//inspiration du tutoriel : http://www.technical-recipes.com/2015/getting-started-with-the-boost-graph-library/#Directed
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
    DjikstraBoost(int newDistNodes, std::vector<std::vector<State>> newConfigurationSpace);
    std::vector<QPoint> getPath(QPoint startCoord, QPoint goalCoord);

private :

    //This method updates the configuration space.
    void setNewConfigurationSpace(std::vector<std::vector<State>> newConfigurationSpace);

    //This method identifies whether the node is free, a hallway or occupied
    State  getNodeState(std::vector<std::vector<State>> newConfigurationSpace,
                      int column,int row,int step);

    //This method generates the vertex list from the configuration space
    void configurationSpaceToVertexList();

    //This method initializes the start and goal nodes
    void initializeStartAndGoal(QPoint startCoord, QPoint goalCoord);

    //this method computes djikstra's shortest path for a given configuration space
    void computeDjikstraShortestPathAlgorithm(QPoint startCoord, QPoint goalCoord);

    //this method adds a new vertex to the graph given it's x, y coordinates
    void addNewVertex(int x, int y);

    //this method generates the edge list from the vertex list
    void vertexListToEdgeList();

    //this method adds a new edge to the graph
    void addEdge(Vertex currentVertex, int currentX, int currentY,
                 int adjacentX, int adjacentY);
    //this method searches for the shortest path using boost library
    void searchForShortestPath();

    //this method reconstructs the path using the boost predecessor list
    void reconstructPath();

    //Graph and Map instanciation
    UndirectedGraph m_myGraph;
    QMap <VertexKey, Vertex>  m_allVertices;  //find the vertex
    QPoint m_startCell, m_goalCell;
    Vertex m_startVertex, m_goalVertex;
    int m_width, m_height, m_num_nodes;
    int m_distNodes;
    std::vector<boost::graph_traits<UndirectedGraph>::vertex_descriptor > m_shortestPath;
    std::vector<QPoint> m_pathCoord;
    std::vector<std::vector<State>> m_configurationSpace;
};

#endif // DJIKSTRABOOST_H

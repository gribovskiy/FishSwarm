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

#define FREE     0 // FIXME : use enum or enum class
#define HALLWAY  1
#define OCCUPIED 2

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
    VertexKey(QPoint const & p): p(p){}
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
// TODO: : change the sections order : 
// (1) public:
// (2) protected: 
// (3) private: 
private :
    // FIXME : need a comment for every method (and for every class), for instance:
    //! This method updates the configuration space. 
    void setNewConfigurationSpace(std::vector<std::vector<int>> newConfigurationSpace);
    int  getNodeState(std::vector<std::vector<int>> newConfigurationSpace,
                      int column,int row,int step);
    void configurationSpaceToVertexList();
    void initializeStartAndGoal(QPoint startCoord, QPoint goalCoord);
    void computeDjikstraShortestPathAlgorithm(QPoint startCoord, QPoint goalCoord);
    void addNewVertex(int x, int y);
    void vertexListToEdgeList();
    void addEdge(Vertex currentVertex, int currentX, int currentY,
                 int adjacentX, int adjacentY);
    void searchForShortestPath();
    void reconstructPath();

    // FIXME : add either _ or m_ before all private members : myGraph => m_myGraph,
    // the same is for other classes.
    //Graph and Map instanciation
    UndirectedGraph myGraph;
    QMap <VertexKey, Vertex>  allVertices;  //find the vertex
    QPoint startCell, goalCell;
    Vertex startVertex, goalVertex;
    int width, height, num_nodes;
    /*static*/ int distNodes;
    std::vector<boost::graph_traits<UndirectedGraph>::vertex_descriptor > shortestPath;
    std::vector<QPoint> pathCoord;
    /*static*/ std::vector<std::vector<int>> configurationSpace;

public:
    DjikstraBoost(int newDistNodes, std::vector<std::vector<int>> newConfigurationSpace);
    std::vector<QPoint> getPath(QPoint startCoord, QPoint goalCoord);
};

#endif // DJIKSTRABOOST_H

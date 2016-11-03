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

#define FREE     0
#define HALLWAY  1
#define OCCUPIED 2

struct vertex_info {
    std::pair<int,int> pos;
};

//Adjacency list for the directed graph
typedef boost::adjacency_list <boost::listS, boost::vecS, boost::undirectedS, vertex_info,
boost::property < boost::edge_weight_t, float > > UndirectedGraph;

typedef boost::graph_traits<UndirectedGraph>::vertex_descriptor Vertex;
typedef std::pair<int, int> Edge;

//http://www.developpez.net/forums/d1298928/c-cpp/bibliotheques/qt/qmap-operator-qpoint/
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
private :

    void setNewConfigurationSpace(std::vector< std::vector<int> > newConfigurationSpace,
                                  int newDistNodes);
    int  getNodeState(std::vector< std::vector<int> > newConfigurationSpace,
                      int column,int row,int step);
    void configurationSpaceToVertexList();
    void initializeStartAndGoal(int startCoord[2], int goalCoord[2]);
    void computeGraph();

    void addVertex(int x, int y);
    void vertexListToEdgeList();
    void vertexNeighborsAndEdges(int currentX, int currentY, Vertex currentVertex);
    void addEdge(Vertex currentVertex, int currentX, int currentY,
                 int adjacentX, int adjacentY);
    void searchForShortestPath();
    void reconstructPath();


    //Graph and Map instanciation
    UndirectedGraph myGraph;
    QMap <VertexKey, Vertex>  allVertices;  //find the vertex
    std::vector< std::vector<int> > configurationSpace;
    int width, height, num_nodes, distNodes;

    std::pair<int,int> startCell, goalCell;
    Vertex startVertex, goalVertex;
    std::vector<boost::graph_traits<UndirectedGraph>::vertex_descriptor > shortestPath;
    std::vector<std::pair <int,int> > pathCoord;

public:
    DjikstraBoost(int newDistNodes, std::vector< std::vector<int> > newConfigurationSpace);
    DjikstraBoost(int startCoord[2], int goalCoord[2], int newDistNodes, std::vector< std::vector<int> > newConfigurationSpace);
    void computeDjikstraShortestPathAlgorithm(int startCoord[2], int goalCoord[2]);
    /*
    DjikstraBoost(int startCoord[2], int goalCoord[2], int distNodes);
    */
    std::vector<std::pair <int,int> > getPath();
};

#endif // DJIKSTRABOOST_H

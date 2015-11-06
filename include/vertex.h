#ifndef VERTEX_H
#define VERTEX_H

#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
class Vertex
{
public:
    struct edge
    {
        float error;
        Vertex* start;
        Vertex* end;
    };
    typedef struct edge* edgeptr;

    Vertex();
    int index;
    edgeptr list;
    std::vector<edgeptr> edges;
    cv::Vec3f pos;
    float var;

    static Vertex* otherEnd(edgeptr e, Vertex* v);
    static void calcError(edgeptr e);
    void insertEdge(edgeptr e);
};

#endif

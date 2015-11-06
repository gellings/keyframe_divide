#ifndef EDGE_H
#define EDGE_H

#include "vertex.h"

class Edge
{

public:
    Edge();
    float error;
    Vertex * start;
    Vertex * end;
};

#endif

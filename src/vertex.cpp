#include "vertex.h"

Vertex::Vertex()
{
    list = NULL;
}

Vertex* Vertex::otherEnd(edgeptr e, Vertex *v)
{
    if(v == e->start)
        return e->end;
    else
        return e->start;
}

void Vertex::calcError(edgeptr e)
{
    cv::Vec3f simpleAve;
    cv::Vec3f butterflyAve;

    simpleAve = (e->start->pos + e->end->pos)/2;

    int divisor = 8;
    butterflyAve = 4*e->start->pos + 4*e->end->pos;

    if(1)
    {
        ;//butterflyAve += 2*  + 2* ;
    }

    butterflyAve /= divisor;

    e->error = cv::norm(simpleAve - butterflyAve);
}

void Vertex::insertEdge(edgeptr e)
{
    if(this->list == NULL)
    {

    }
}

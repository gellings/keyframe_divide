#ifndef PROCESS_EDGES
#define PROCESS_EDGES

#include "segment-graph.h"
#include <iostream>
#include <vector>
#include <map>

long getid(int a, int b)
{
    long al = a;
    long bl = b;
    if(a > b)
        return (al + bl)*(al + bl + 1)/2 + bl;
    else
        return (al + bl)*(al + bl + 1)/2 + al;
}

class process_edges
{
public:
    process_edges(int w, int h){height = h; width = w;}
    void addEdge(edge e, int compIdA, int compIdB);
    void orderEdges();
    std::map<long,std::vector<edge> > e_map;
private:
    int height;
    int width;

    int rot(int dir, bool cw);
};

void process_edges::addEdge(edge e, int compIdA, int compIdB)
{
    long id = getid(compIdA, compIdB);
    e_map[id].push_back(e);
}

int searchVec(std::vector<edge>* v, int a, int b)
{
    for(int i=0;i<v->size();i++)
    {
        if((v->at(i).a == a && v->at(i).b == b) || (v->at(i).a == b && v->at(i).b == a))
            return i;
    }
    return -1;
}
void mv(std::vector<edge>* nV, std::vector<edge>* oV, int idx, bool dir)
{
    if(dir)
        nV->push_back(oV->at(idx));
    else
        nV->insert(nV->begin(),oV->at(idx));

    oV->erase(oV->begin() + idx);
}

int process_edges::rot(int dir, bool cw)
{

    if(dir == 1)
        return (cw ? width : -width);
    else if(dir == width)
        return (cw ? -1 : 1);
    else if(dir == -1)
        return (cw ? -width : width);
    else
        return (cw ? 1 : -1);
}

void process_edges::orderEdges()
{
    //std::map<long, std::vector<edge> >::iterator it = e_map.begin();
    for(std::map<long, std::vector<edge> >::iterator it = e_map.begin(); it != e_map.end(); it++)
    {
        std::vector<edge>* newVec, *oldVec;
        newVec = new std::vector<edge>;
        oldVec = &(it->second);
        int n = oldVec->size();
        //std::cout << n = oldVec->size() << " ";
        //newVec.push_back(oldVec[0]);

        int dir1(0), dir2(0);

        while(oldVec->size() > 0)
        {
            if(dir1 == 0 && dir2 == 0)
            {
                if(newVec->size() != 0)
                {   edge e; e.a = -1; e.b = -1; newVec->push_back(e);  }
                if(oldVec->at(0).a == (oldVec->at(0).b - width))  //edge is up down
                {   dir1 = 1; dir2 = -1;   }
                else if(oldVec->at(0).a == (oldVec->at(0).b - 1)) //edge is right left
                {   dir1 = width; dir2 = -width;    }
                mv(newVec, oldVec, 0, true);
            }

            if(dir1 != 0)
            {
                int r;
                if((r = searchVec(oldVec, newVec->at(newVec->size()-1).a + dir1, newVec->at(newVec->size()-1).b + dir1)) > -1)
                    mv(newVec, oldVec, r, true);
                else if((r = searchVec(oldVec, newVec->at(newVec->size()-1).a, newVec->at(newVec->size()-1).a + dir1)) > -1)
                {   mv(newVec, oldVec, r, true); dir1 = rot(dir1,(dir1==1||dir1==-width?false:true));   }
                else if((r = searchVec(oldVec, newVec->at(newVec->size()-1).b, newVec->at(newVec->size()-1).b + dir1)) > -1)
                {   mv(newVec, oldVec, r, true); dir1 = rot(dir1,(dir1==1||dir1==-width?true:false));  }
                else
                    dir1 = 0;
            }

            if(dir2 != 0)
            {
                int r;
                if((r = searchVec(oldVec, newVec->at(0).a + dir2, newVec->at(0).b + dir2)) > -1)
                    mv(newVec, oldVec, r, false);
                else if((r = searchVec(oldVec, newVec->at(0).a, newVec->at(0).a + dir2)) > -1)
                {   mv(newVec, oldVec, r, false); dir2 = rot(dir2,(dir2==1||dir2==-width?false:true));   }
                else if((r = searchVec(oldVec, newVec->at(0).b, newVec->at(0).b + dir2)) > -1)
                {   mv(newVec, oldVec, r, false); dir2 = rot(dir2,(dir2==1||dir2==-width?true:false));   }
                else
                    dir2 = 0;
            }
        }
        it->second = *newVec;
    }
}

#endif

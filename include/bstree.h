#include <iostream>
#include<cctype>
#include <stdlib.h>

#include "vertex.h"

using namespace std;

class bstree
{
private:
    struct node
    {
        Vertex::edgeptr edge;
        node *left;
        node *right;
        int height;
    };
    typedef struct node* nodeptr;

    nodeptr root;
    void insert(Vertex::edgeptr edge, nodeptr &);
    bool del(Vertex::edgeptr, nodeptr &);
    Vertex::edgeptr deletemin(nodeptr &);
    Vertex::edgeptr findmin(nodeptr);
    Vertex::edgeptr findmax(nodeptr);
    nodeptr find(Vertex::edgeptr edge, nodeptr &);
    nodeptr srl(nodeptr &);
    nodeptr drl(nodeptr &);
    nodeptr srr(nodeptr &);
    nodeptr drr(nodeptr &);

    public:
        void insert(Vertex::edgeptr edge);
        bool del(Vertex::edgeptr);
        Vertex::edgeptr findmin();
        Vertex::edgeptr findmax();
        void makeempty(nodeptr &);
        void copy(nodeptr &,nodeptr &);
        nodeptr nodecopy(nodeptr &);
        void preorder(nodeptr);
        void inorder(nodeptr);
        void postorder(nodeptr);
        int bsheight(nodeptr);

        int max(int,int);
        int nonodes(nodeptr);
};

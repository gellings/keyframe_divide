#include "bstree.h"

// Inserting a node
void bstree::insert(Vertex::edgeptr edge)
{
    insert(edge, this->root);
}

void bstree::insert(Vertex::edgeptr edge, nodeptr &p)
{
    if (p == NULL)
    {
        p = new node;
        p->edge = edge;
        p->left=NULL;
        p->right = NULL;
        p->height=0;
        if (p==NULL)
        {
            cout<<"Out of Space\n"<<endl;
        }
    }
    else
    {
        if (edge->error < p->edge->error)
        {
            insert(edge,p->left);
            if ((bsheight(p->left) - bsheight(p->right))==2)
            {
                if (edge->error < p->left->edge->error)
                {
                    p = srl(p);
                }
                else
                {
                    p = drl(p);
                }
            }
        }
        else //if (edge->error > p->edge->error)
        {
            insert(edge, p->right);
            if ((bsheight(p->right) - bsheight(p->left))==2)
            {
                if (edge->error > p->right->edge->error)
                {
                    p=srr(p);
                }
                else
                {
                    p = drr(p);
                }
            }
        }
//        else
//        {
//            cout<<"Element Exists\n"<<endl;
//        }
    }
    int m,n,d;
    m=bsheight(p->left);
    n=bsheight(p->right);
    d=max(m,n);
    p->height = d + 1;
}
// Finding the Smallest
Vertex::edgeptr bstree::findmin()
{
    return findmin(root);
}

Vertex::edgeptr bstree::findmin(nodeptr p)
{
    if (p==NULL)
    {
        return NULL;
    }
    else
    {
        while(p->left != NULL)
        {
            p=p->left;
        }
        return p->edge;
    }
}
// Finding the Largest node
Vertex::edgeptr bstree::findmax()
{
    return findmax(root);
}

Vertex::edgeptr bstree::findmax(nodeptr p)
{
    if (p==NULL)
    {
        return NULL;
    }
    else
    {
        while(p->right !=NULL)
        {
            p=p->right;
        }
        return p->edge;
    }
}
// Finding an element
bstree::nodeptr bstree::find(Vertex::edgeptr edge, nodeptr &p)
{
    if (p != NULL)
    {
        if(edge == p->edge)
        {
            return p;
        }
        else if (edge->error < p->edge->error)
        {
            find(edge,p->left);
        }
        else
        {
            find(edge,p->right);
        }
    }
    return NULL;
}
// Copy a tree
void bstree::copy(nodeptr &p,nodeptr &p1)
{
    makeempty(p1);
    p1 = nodecopy(p);
}
// Make a tree empty
void bstree::makeempty(nodeptr &p)
{
    nodeptr d;
    if (p != NULL)
    {
        makeempty(p->left);
        makeempty(p->right);
        d=p;
        free(d);
        p=NULL;
    }
}
// Copy the nodes
bstree::nodeptr bstree::nodecopy(nodeptr &p)
{
    nodeptr temp;
    if (p==NULL)
    {
        return p;
    }
    else
    {
        temp = new node;
        temp->edge = p->edge;
        temp->left = nodecopy(p->left);
        temp->right = nodecopy(p->right);
        return temp;
    }
}

// Deleting a node
bool bstree::del(Vertex::edgeptr edge)
{
    return del(edge, root);
}

bool bstree::del(Vertex::edgeptr edge,nodeptr &p)
{
    nodeptr d;
    if (p==NULL)
    {
        return false;
    }
    else if (edge->error < p->edge->error)
    {
        del(edge,p->left);
    }
    else if (edge->error > p->edge->error)
    {
        del(edge,p->right);
    }
    else if ((p->left == NULL) && (p->right == NULL))
    {
        d=p;
        free(d);
        p=NULL;
        return true;
    }
    else if (p->left == NULL)
    {
        d=p;
        free(d);
        p=p->right;
        return true;
    }
    else if (p->right == NULL)
    {
        d=p;
        p=p->left;
        free(d);
        return true;
    }
    else
    {
        p->edge = deletemin(p->right);
    }
    return false;
}

Vertex::edgeptr bstree::deletemin(nodeptr &p)
{
    Vertex::edgeptr c;
    //cout<<"inside deltemin\n"<<endl;
    if (p->left == NULL)
    {
        c=p->edge;
        p=p->right;
        return c;
    }
    else
    {
        c=deletemin(p->left);
        return c;
    }
}
void bstree::preorder(nodeptr p)
{
    if (p!=NULL)
    {
        cout << p->edge->error << "\t";
        preorder(p->left);
        preorder(p->right);
    }
}

// Inorder Printing
void bstree::inorder(nodeptr p)
{
    if (p!=NULL)
    {
        inorder(p->left);
        cout << p->edge->error << "\t";
        inorder(p->right);
    }
}

// PostOrder Printing
void bstree::postorder(nodeptr p)
{
    if (p!=NULL)
    {
        postorder(p->left);
        postorder(p->right);
        cout << p->edge->error << "\t";
    }
}

int bstree::max(int value1, int value2)
{
    return ((value1 > value2) ? value1 : value2);
}
int bstree::bsheight(nodeptr p)
{
    int t;
    if (p == NULL)
    {
        return -1;
    }
    else
    {
        t = p->height;
        return t;
    }
}

bstree::nodeptr bstree:: srl(nodeptr &p1)
{
    nodeptr p2;
    p2 = p1->left;
    p1->left = p2->right;
    p2->right = p1;
    p1->height = max(bsheight(p1->left),bsheight(p1->right)) + 1;
    p2->height = max(bsheight(p2->left),p1->height) + 1;
    return p2;
}
bstree::nodeptr bstree:: srr(nodeptr &p1)
{
    nodeptr p2;
    p2 = p1->right;
    p1->right = p2->left;
    p2->left = p1;
    p1->height = max(bsheight(p1->left),bsheight(p1->right)) + 1;
    p2->height = max(p1->height,bsheight(p2->right)) + 1;
    return p2;
}
bstree::nodeptr bstree:: drl(nodeptr &p1)
{
    p1->left=srr(p1->left);
    return srl(p1);
}
bstree::nodeptr bstree::drr(nodeptr &p1)
{
    p1->right = srl(p1->right);
    return srr(p1);
}

int bstree::nonodes(nodeptr p)
{
    int count=0;
    if (p!=NULL)
    {
        nonodes(p->left);
        nonodes(p->right);
        count++;
    }
    return count;
}

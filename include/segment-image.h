/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#ifndef SEGMENT_IMAGE
#define SEGMENT_IMAGE

#include <cstdlib>
#include "image.h"
#include "misc.h"
#include "filter.h"
#include "segment-graph.h"
#include "process-edges.h"

// random color
rgb random_rgb(){ 
  rgb c;
  double r;
  
  c.r = (uchar)random();
  c.g = (uchar)random();
  c.b = (uchar)random();

  return c;
}

// dissimilarity measure between pixels
static inline float diff(image<float> *r, image<float> *g, image<float> *b, image<float> *d, image<float> *u,
			 int x1, int y1, int x2, int y2) {
    if(1 || imRef(d, x1, y1) != imRef(d, x1, y1) || imRef(d, x2, y2) != imRef(d, x2, y2))
        return sqrt(square(imRef(r, x1, y1)-imRef(r, x2, y2)) +
                square(imRef(g, x1, y1)-imRef(g, x2, y2)) +
                square(imRef(b, x1, y1)-imRef(b, x2, y2)));
    else
        return sqrt(square(imRef(r, x1, y1)-imRef(r, x2, y2)) +
              square(imRef(g, x1, y1)-imRef(g, x2, y2)) +
              square(imRef(b, x1, y1)-imRef(b, x2, y2)) +
              square(imRef(d, x1, y1)-imRef(d, x2, y2))/square((u, x1, y1)+imRef(u, x2, y2)));
}

/*
 * Segment an image
 *
 * Returns a color image representing the segmentation.
 *
 * im: image to segment.
 * sigma: to smooth the image.
 * c: constant for treshold function.
 * min_size: minimum component size (enforced by post-processing stage).
 * num_ccs: number of connected components in the segmentation.
 */
image<rgb> *segment_image(image<rgbdu> *im, float sigma, float c, int min_size,
			  int *num_ccs) {
  int width = im->width();
  int height = im->height();

  image<float> *r = new image<float>(width, height);
  image<float> *g = new image<float>(width, height);
  image<float> *b = new image<float>(width, height);
  image<float> *d = new image<float>(width, height);
  image<float> *un = new image<float>(width, height);

  // smooth each color channel  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(r, x, y) = imRef(im, x, y).r;
      imRef(g, x, y) = imRef(im, x, y).g;
      imRef(b, x, y) = imRef(im, x, y).b;
      imRef(d, x, y) = imRef(im, x, y).d;
      imRef(un, x, y) = imRef(im, x, y).u;
    }
  }
  image<float> *smooth_r = smooth(r, sigma);
  image<float> *smooth_g = smooth(g, sigma);
  image<float> *smooth_b = smooth(b, sigma);
  delete r;
  delete g;
  delete b;
 
  // build graph
  edge *edges = new edge[width*height*2];
  int num = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (x < width-1) {
	edges[num].a = y * width + x;
	edges[num].b = y * width + (x+1);
    edges[num].w = diff(smooth_r, smooth_g, smooth_b, d, un, x, y, x+1, y);
	num++;
      }

      if (y < height-1) {
	edges[num].a = y * width + x;
	edges[num].b = (y+1) * width + x;
    edges[num].w = diff(smooth_r, smooth_g, smooth_b, d, un, x, y, x, y+1);
	num++;
      }

//      if ((x < width-1) && (y < height-1)) {
//	edges[num].a = y * width + x;
//	edges[num].b = (y+1) * width + (x+1);
//	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y+1);
//	num++;
//      }

//      if ((x < width-1) && (y > 0)) {
//	edges[num].a = y * width + x;
//	edges[num].b = (y-1) * width + (x+1);
//	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y-1);
//	num++;
//      }
    }
  }
  delete smooth_r;
  delete smooth_g;
  delete smooth_b;

  // segment
  universe *u = segment_graph(width*height, num, edges, c);
  process_edges pe(width, height);

  // post process small components
  for (int i = 0; i < num; i++) {
    int a = u->find(edges[i].a);
    int b = u->find(edges[i].b);
    if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
      u->join(a, b);
  }

  // add edges to edge processer
  for (int i = 0; i < num; i++) {
    int a = u->find(edges[i].a);
    int b = u->find(edges[i].b);
    if(a != b)
        pe.addEdge(edges[i],a,b);
  }

  pe.orderEdges();
  delete [] edges;
  *num_ccs = u->num_sets();

  image<rgb> *output = new image<rgb>(width, height);

  // pick random colors for each component
  rgb *colors = new rgb[width*height];
  for (int i = 0; i < width*height; i++)
    colors[i] = random_rgb();
  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int comp = u->find(y * width + x);
      imRef(output, x, y) = colors[comp];
    }
  }

  rgb *red = new rgb();
  red->r = (uchar)255;
  red->g = (uchar)255;
  red->b = (uchar)255;
  rgb *bl = new rgb();
  bl->r = 0;
  bl->g = 0;
  bl->b = 0;
  int co = 0;
  //std::map<long, std::vector<edge> >::iterator it = pe.e_map.begin();
  for (std::map<long, std::vector<edge> >::iterator it = pe.e_map.begin(); it != pe.e_map.end(); it++)
  {co++;
        if(0)
            for(int i=0;i<it->second.size();i++)
            {
                int x = it->second.at(0).a % width;
                int y = it->second.at(0).a / width;
                imRef(output, x, y) = *bl;
                x = it->second.at(0).b % width;
                y = it->second.at(0).b / width;
                imRef(output, x, y) = *bl;
                x = it->second.at(i).b % width;
                y = it->second.at(i).b / width;
                imRef(output, x, y) = *bl;
                //std::cout << i << std::endl;
            }
  }
  delete red;

  delete [] colors;  
  delete u;

  return output;
}

#endif

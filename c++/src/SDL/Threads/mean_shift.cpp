//
// mean shift clustering
//
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <vector>
#include "Vector3.h"
#include "Matrix3.h"
//#include <pthread.h>
#include <SDL/SDL_thread.h>

#include <SDL/SDL.h>
#define GL_GLEXT_PROTOTYPES
#include <SDL/SDL_opengl.h>
#define GLX_GLXEXT_PROTOTYPES
#include <GL/glx.h>
#include <GL/glxext.h>

using namespace std;

void plot_points(std::vector<v3d> &points, const v3d &color){
  glColor4d(color.r(), color.g(), color.b(), 1.0);
  glBegin(GL_POINTS);
  for (unsigned i=0; i<points.size(); i++){
    glVertex3d(points.at(i).x(), points.at(i).y(), points.at(i).z());
  }
  glEnd();
  SDL_GL_SwapBuffers();
}

void plot_track(std::vector<v3d> &points, const v3d &color){
  glColor4d(color.r(), color.g(), color.b(), 0.4);
  glBegin(GL_LINES);
  unsigned i;
  glVertex3d(points.at(0).x(), points.at(0).y(), points.at(0).z()+1);
  for (i=1; i<points.size(); i++){
    glVertex3d(points.at(i).x(), points.at(i).y(), points.at(i).z()+1);
    glVertex3d(points.at(i).x(), points.at(i).y(), points.at(i).z()+1);
  }
  glVertex3d(points.at(i-1).x(), points.at(i-1).y(), points.at(i-1).z()+1);
  glEnd();
  SDL_GL_SwapBuffers();
}


void sample_gaussian(std::vector<v3d> & points, 
		     double cx, double cy, double sigma, int n){
  for (int i=0; i<n; i++){
    double r = sigma * double(rand())/RAND_MAX;
    double th = 2*M_PI*double(rand())/RAND_MAX;
    double x = cx + r * cos(th);
    double y = cy + r * sin(th);
    double z = 0.;
    points.push_back(v3d(x,y,z));
  }

  int npts = 100;
  glBegin(GL_LINES);
  glColor4d(1., 0., 0., 1.);
  for (int i=0; i<npts; i++){
    double r = sigma/5;
    double th = 2*M_PI * i / (npts-1);
    double x = cx + r * cos(th);
    double y = cy + r * sin(th);
    double z = 0.;
    glVertex3d(x, y, z);
  }
  glEnd();
}


inline double kernel(double x){
  const double h_inv = 1./0.04;
  return exp(-x*h_inv);
}


void mean_shift(std::vector<v3d> &points)
{

  int n_starts = 10000;
  std::vector<v3d> centers;
  std::vector<v3d> colors;

  //#define RANDOM
#ifdef RANDOM
  for (int i=0; i<n_starts; i++){
    v3d x(3.*double(rand())/RAND_MAX,
	  3.*double(rand())/RAND_MAX,
	  3.*double(rand())/RAND_MAX);
#else
    glPointSize(4.0);


  int N = 70;
  for (int i=0; i<N;i++){
    for (int j=0; j<N; j++){
      v3d x(3.*i/N, 3.*j/N, 0);
#endif

    std::vector<v3d> path;
    bool converged = false;
    double thresh = 0.000001;
    do {
      v3d num(0., 0., 0.);
      double denom = 0.;
      
      for (unsigned j=0; j<points.size(); j++){
	v3d d = x - points.at(j);
	double w = kernel(d.dot(d));
	num += w * points.at(j);
	denom += w;
      }
      v3d x_new = num *(1./denom);
      double delta = (x_new - x).length();
      if (delta < thresh) converged = true;
      path.push_back(x);
      x = x_new;
    } while (!converged);
    path.push_back(x);

    bool already_found = false;
    for (unsigned j=0; j<centers.size(); j++){
      double found_thresh = thresh*20.;
      if ((x - centers.at(j)).length() < found_thresh){
	already_found = true;
	centers.at(j) = 0.5 * (centers.at(j) + x);
	plot_track(path, colors.at(j));
      }
    }
    if (!already_found){
      centers.push_back(x);
      v3d color;
      double min_dist;
      do {
	color = v3d(0.5+0.5*double(rand())/RAND_MAX,
		    0.5+0.5*double(rand())/RAND_MAX,
		    0.5+0.5*double(rand())/RAND_MAX);
	m3d saturator(0.976881,  -0.148714,  -0.028881,
		      -0.198943,   0.662065,  -0.075851,
		      -0.157366,  -0.308943,   0.992632);
	color = saturator * color;
	if (colors.size() == 0) break;
	min_dist = 10.;
	for (unsigned j=0; j<colors.size(); j++){
	  double dist = (colors.at(j) - color).length();
	  if (dist < min_dist) min_dist = dist;
	}
      } while (min_dist < 0.1);

      colors.push_back(color);
      plot_track(path, color);
      printf ("%f %f %f\n", x.x(), x.y(), x.z());
      int npts = 100;

      glBegin(GL_LINES);
      glColor4d(1., 1., 0., 1.);
      for (int i=0; i<npts; i++){
	double r = 0.05;
	double th = 2*M_PI * i / (npts-1);
	double xp = x.x() + r * cos(th);
	double yp = x.y() + r * sin(th);
	double zp = 0.;
	glVertex3d(xp, yp, zp);
      }
      glEnd();
    }
    }
#ifndef RANDOM
}
#endif


  printf("-----------------\n");
  for (unsigned j=0; j<centers.size(); j++){
    printf ("%f %f %f\n", centers.at(j).x(), 
	    centers.at(j).y(), 
	    centers.at(j).z());
  }

}

void SDL()
{
  SDL_Surface *screen;
  if ( SDL_Init(SDL_INIT_VIDEO) < 0 ) {
    fprintf(stderr, "Unable to init SDL: %s\n", SDL_GetError());
    exit(1);
  }
  atexit(SDL_Quit);
  
  SDL_GL_SetAttribute(SDL_GL_RED_SIZE,     8);
  SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE,   8);
  SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE,    8);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  
  screen = NULL;
  int width = 1024;
  int height = 1024;
  if (NULL == (screen = SDL_SetVideoMode(width, height, 0, 
					 SDL_OPENGL))){
    fprintf(stderr, "Can't set OpenGL mode: %s\n", SDL_GetError());
    SDL_Quit();
    exit(1);
  }
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, 3, 3, 0, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity(); 
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glLineWidth(2.0);
  glPointSize(2.0);
}

 int event_thread(void *arg){
  bool done = false;
  SDL_Event event;
  while (!done)
  {
    while (!done && SDL_WaitEvent(&event)){
      switch (event.type)
	 {
      case SDL_KEYDOWN:
        switch(event.key.keysym.sym)
	   {
		case SDLK_ESCAPE:
			done = true;
			break;
		default:
			break;
        }
        break;

      case SDL_QUIT:
        done = true;
        break;
      }
    }
  }
  exit(0);
 }


int main(int argc, char **argv){
  std::vector<v3d> points;
  srand(2764);

  atexit(SDL_Quit);
  SDL();

  for (int i=0; i<10; i++){
    sample_gaussian(points,
		    0.5+2*double(rand())/RAND_MAX,
		    0.5+2*double(rand())/RAND_MAX,
		    0.25 + 1.0* double(rand())/RAND_MAX,
		    200 + rand()% 1000);
  }
  
  plot_points(points, v3d(0.25, 0.25, 1.));

  SDL_Thread *thread;
  thread = SDL_CreateThread(event_thread, NULL);
  mean_shift(points);

  return 0;
}

#ifndef SDL_HELPER_H
#define SDL_HELPER_H


#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#include <string>
#include <iostream>
#include <assert.h>
#include <GL/glut.h>

using namespace std;

SDL_Surface * InitSDL(int width, int height, string Title);
SDL_Surface *load_image(string filename );
void polygon(int a, int b, int c , int d);
void DrawCubeName();
void DrawPoint(double x, double y, double z);
void DrawCube();

#endif
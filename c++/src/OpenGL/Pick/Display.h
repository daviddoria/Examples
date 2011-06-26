#ifndef DISPLAY_H
#define DISPLAY_H

#include <geom_Point3.h>
#include <geom_Vector3.h>
#include <geom_Ray3.h>
#include <geom_Triangle3.h>
#include <glut.h>

#include <iostream>
#include <string>

#define DEFAULTWIDTH 5
#define DEFAULTCOLOR 'w'

void DrawPoint(const geom_Point3 &P, const double rad = .4, const char Color = DEFAULTCOLOR);
void DrawLine(const geom_Point3 &P1, const geom_Point3 &P2, const double width = DEFAULTWIDTH, const char Color = DEFAULTCOLOR);
void DrawVector(const geom_Vector3 &V, const double length, const double width = DEFAULTWIDTH, const char Color = DEFAULTCOLOR);
void DrawRay(const geom_Ray3 &R, const double length, const double width = DEFAULTWIDTH, const char Color = DEFAULTCOLOR);
void DrawTriangle(const geom_Point3 &P1, const geom_Point3 &P2, const geom_Point3 &P3, const double width = DEFAULTWIDTH, const char Color = DEFAULTCOLOR);
void DrawTriangle(const geom_Triangle3 &T, const double width = DEFAULTWIDTH, const char Color = DEFAULTCOLOR);
void DrawCube();

void DrawAxes(const double AxisLength = 4);
void DrawGrid(const string WhichGrid, const int Size, const double width = DEFAULTWIDTH, const char Color = DEFAULTCOLOR);

void SetColor(const char Color);
void WriteText(float x, float y, float z, string Text);
#endif
#ifndef __DUBINS_H__
#define __DUBINS_H__

#include <iostream>
#include <tuple>
#include <cmath>
#include <string>
#include <assert.h>	
#include "../clipper/cpp/clipper.hpp"
#include "../Utils/utils.h"

/*
 * Compile command:
  g++ prova.cpp `pkg-config opencv --cflags --libs` 
*/

using namespace cv;
using namespace std;

struct arc{
	double x0, y0, th0, k, L, xf, yf, thf;
}typedef arc;

struct curve{
	arc a1,a2,a3;
	double L;
}typedef curve;

tuple <int, curve> dubins(double, double, double, double, double, double, double);
arc dubinsarc(double, double, double, double, double);
curve dubinscurve(double, double, double, double, double, double, double, double, double);
tuple <double, double, double, double> scaleToStandard(double, double, double, double, double, double, double);
tuple <double, double, double> scaleFromStandard(double, double, double, double);
tuple <bool, double, double, double> LSL(double, double, double);
tuple <bool, double, double, double> RSR(double, double, double);
tuple <bool, double, double, double> LSR(double, double, double);
tuple <bool, double, double, double> RSL(double, double, double);
tuple <bool, double, double, double> RLR(double, double, double);
tuple <bool, double, double, double> LRL(double, double, double);
tuple <double, double, double> circline(double, double, double, double, double);
Mat plotarc(arc, string, Mat);
Mat plotdubins(curve, string, string, string, Mat );
void python_plot(curve, string, string, string);
tuple <point_list *, double_list *> intersLineLine(double, double, double, double, double, double, double, double);
double cross2D(double *, double *);
double dot2D(double *, double *);
tuple <point_list *, double_list *> intersCircleLine(double, double, double, double, double, double, double);

#endif
#ifndef __DUBINS_H__
#define __DUBINS_H__

#include <iostream>
#include <tuple>
#include <cmath>
#include <string>
#include <assert.h>
#include "../Utility/utility.h"
#include "../Roadmap/roadmap.h"
#include "../Robot_manager/robots.h"
#include "../config.hpp"

/*
 * Compile command:
  g++ prova.cpp `pkg-config opencv --cflags --libs`
*/

using namespace cv;
using namespace std;

/**
 * \struct
 * This structure represents an arc of a dubins curve.
 * The attributes are:
 * @param double x0: x-starting coordinate of the arc
 * @param double y0: y-starting coordinate of the arc
 * @param double th0: Starting angle of the arc
 * @param double k: Angle curvature
 * @param double L: Length of the arc
 * @param double xf: x-final coordinate of the arc
 * @param double yf: y-final coordinate of the arc
 * @param double thf: Final angle of the arc
 * @param double xc: x-center coordinate of the arc
 * @param double yc: y-center coordinate of the arc
 * @param double r: radius of the arc
 */
// k = angle curvature -> 1/k radius curvature
struct arc
{
	double x0, y0, th0, k, L, xf, yf, thf, xc, yc, r;
} typedef arc;

/**
 * \struct
 * This structure represents a dubins curve
 * The attributes are:
 * @param arc a1: First arc
 * @param arc a2: Second arc
 * @param arc a3: Third arc
 * @param double L: Curve's length
 */
struct curve
{
	arc a1, a2, a3;
	double L; // Curve length
} typedef curve;

tuple<int, curve> dubins(double, double, double, double, double, double, double);
arc dubinsarc(double, double, double, double, double);
curve dubinscurve(double, double, double, double, double, double, double, double, double);
tuple<double, double, double, double> scaleToStandard(double, double, double, double, double, double, double);
tuple<double, double, double> scaleFromStandard(double, double, double, double);
tuple<bool, double, double, double> LSL(double, double, double);
tuple<bool, double, double, double> RSR(double, double, double);
tuple<bool, double, double, double> LSR(double, double, double);
tuple<bool, double, double, double> RSL(double, double, double);
tuple<bool, double, double, double> RLR(double, double, double);
tuple<bool, double, double, double> LRL(double, double, double);
tuple<double, double, double, double, double, double> circline(double, double, double, double, double, bool);

/**
 * \fun plotarc(arc a, string c, Mat img)
 * This function plot a given arc in a given img.
 * @param arc a: Arc to plot
 * @param string c: Color of the plot
 * @param Mat img: img where to plot
 * @return Mat: updated img
 */
Mat plotarc(arc, string, Mat);

/**
 * \fun plotfubins(curve d, string c1, string c2, string c3, Mat arena)
 * This function plot a given curve in a given img.
 * @param curve d: Curve to plot
 * @param string c1: Color of the first arc of the curve
 * @param string c2: Color of the second arc of the curve
 * @param string c3: Color of the third arc of the curve
 * @param Mat arena: Img where to plot the curve.
 * @return Mat: Updated img with the curve plotted
 */
Mat plotdubins(curve, string, string, string, Mat);


/**
 * \fun tuple<double, double, double> get_circle_center(double x1, double y1, double x2, double y2, double x3, double y3)
 * This function return the center of a circle given three points of it
 * @param double x1: x-coordinate of the first point of the circonference
 * @param double y1: y-coordinate of the first point of the circonference
 * @param double x2: x-coordinate of the second point of the circonference
 * @param double y2: y-coordinate of the second point of the circonference
 * @param double x3: x-coordinate of the third point of the circonference
 * @param double y3: y-coordinate of the third point of the circonference
 * @returns tuple<double h, double k, double r>: x,y coordinates of the center and the circle radius 
 */
tuple<double, double, double> get_circle_center(double, double, double, double, double, double);

/**
 * \fun tuple<curve, int> dubins_no_inter(double x0, double y0, double th0,
								  double xf, double yf, double *thf,
								  double Kmax, point_list *arena_limits, polygon *obstacle,
								  double search_angle, vector<double> used_th)
 * This function return a dubin curve that doesn't intersect with any object.
 * @param double x0: x-starting coordinate of the dubins curve
 * @param double y0: y-starting coordinate of the dubins curve
 * @param double th0: starting angle of the dubins curve
 * @param double xf: x-final coordinate of the dubins curve
 * @param double yf: x-final coordinate of the dubins curve
 * @param double thf: Ending angle of the dubins curve
 * @param double kmax: Max angle curvature possible
 * @param point_list *arena_limits: Points of the arena 
 * @param polygon *obstacle: First node of the obstacle list in the arena
 * @param double search_angle: Range where to search the dubins curve with minimum distance
 * @param vector<double> used_th: List of the already used final angle that must not to be re-used
 * @returns tuple<curve c, int pidx>: Dounded curve c and it's pidx. If pidx <= 0 no curve is founded
 */
tuple<curve, int> dubins_no_inter(double, double, double, double, double, double *, double,
								  point_list*, polygon*, double, vector<double>);
			
/**
 * \fun bool pt_in_arc(point_node *ptso, arc a)
 * This function check if the intersection points founded are included in the arc used for a dubins curve
 * @see intersCircleLine: See the intersection points between a circle and a line. 
 * 
 * @param point_node *ptso: Interection points to check if they are in the arc
 * @param arc a: Given arc
 * @return bool: Say if there is at least one point of intersection
 */
bool pt_in_arc(point_node *, arc);

/**
 * \fun bool find_intersection(arc a, point_node *pnt, point_node *pnt_next)
 * This function search for an intersection between a circle and a line.
 * @param arc a: Given arc
 * @param point_node *pnt: First point of the line
 * @param point_node *pnt_next: Second point of the line
 * @return Bool: Say if there is and intersection or not.
 */
bool find_intersection(arc, point_node *, point_node *);

/**
 * \fun vector<double> opti_theta(vector<double> xpath, vector<double> ypath)
 * This function calculate the optimized arriving angle for the dubins curves for all the path to follow
 * @param vecotr<double> xpath: x-coordinates of the centroids of the free space cells of the path to follow 
 * @param ypath<double> ypath: y-coordinates of the centroids of the free space cells of the path to follow
 * @return vector<double>: Optimized arriving angle
 */
vector<double> opti_theta(vector<double> xpath, vector<double> ypath);

/**
 * \fun Pose get_pose(arc a)
 * This function convert the given arc in a Pose object for the simulator
 * @param arc a: Given arc
 * @return Pose 
 */
Pose get_pose(arc);

/**
 * \fun Path push_path(curve c, Path p)
 * This function push the Pose in a Path vector that the simulator will use to move the robots
 * @param curve c: Curve that must be converted into poses before pushing it into path
 * @param Path p: Path where to push the poses
 * @return Path: Updated Path
 */
Path push_path(curve, Path);

/**
 * \fun vector<curve> get_dubins_path(points_map arena, World_representation abstract_arena, Robot *r)
 * This function return the dubins path formed by all the dubins curve that a robot must follow.
 * @param points_map arena: Given arena
 * @param World_representation abstract_arena: Abstract representation of the arena
 * @param Robot *r: Robot for which the dubins path must be obtained
 * @return vector<curve>: Dubins path for the given robot 
 */
vector<curve> get_dubins_path(points_map, World_representation, Robot *);

/**
 * \fun vector<double> theta_discretization(double starting_angle, double search_angle)
 * This function discretizes the area around an angle
 * @param double starting_angle: Center angle of the area to discretize
 * @param double search_angle: Area to discretize around the starting_angle 
 * @return vector<double>: All the discretized angles
 */
vector<double> theta_discretization(double, double);

#endif

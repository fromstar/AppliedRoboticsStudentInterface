#ifndef __UTILITY_H__
#define __UTILITY_H__

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <string.h>

using pt = boost::geometry::model::d2::point_xy<double>;
using Polygon_boost = boost::geometry::model::polygon<pt>;

using namespace cv;
using namespace std;

struct double_node{
	double value;
	double_node *pnext = NULL;
	
	double_node(double a){
		value = a;
	}
}typedef double_node;

struct double_list{
	double_node *head = NULL;
	double_node *tail = NULL;
	int size = 0;
	
	void add_node(double_node *);
	void delete_list();
	void print_list();
	
}typedef double_list;

struct point_node{
	double x;
	double y;
	point_node *pnext = NULL;
	
	point_node(double a, double b){
		x=a;
		y=b;
		}
	point_node* copy();
	void Print();
}typedef point_node;

struct point_list{
	point_node *head = NULL;
	point_node *tail = NULL;
	double x_min, y_min;
	double x_max,y_max;
	int size = 0;
	
	void add_node(point_node *, int iterations=1);
	void append_list(point_list *e);
	void print_list();
	void delete_list();	
}typedef point_list;

/**
 * \struct
 * Edge struct. Represents the edges inside a polygon.
 * Attributes:
 * @param points : pointer of type point_list. Contains two points of the
 * 				   edge.
 * @param next : pointer of type Edge. Points to the next instance.
 * @param slope : pointer of type double. The slope of the edge.
 *
 * Available methods are:
 * @see info(): It prints the coordinates of the points constituing an edge and
 * its slope.
 * @see draw(): It allows to draw the edge inside a Mat image object.
 * @see intersection(Edge): It checks whether the edge intersects with another.
 */
typedef struct Edge{
	point_list* points;
	double *slope = new double;
	Edge *next = NULL;

	// constructors
	Edge(point_node *p_1, point_node *p_2);

	// methods
	void info();
	void draw(cv::Mat img, cv::Scalar color=cv::Scalar(255, 255, 255),
			  int thickness=1);
	point_node* intersection(Edge *e);
	~Edge();
}Edge;

/**
 * \struct
 * This struct implements a list of edges.
 * It is constituted by:
 * @param head: Edge*. It is the head of the list.
 * @param tail: Edge*. It is the tail of the list.
 * The methods available are:
 * @see add_edge(Edge): allows to add an edge to the list.
 * @see info(): Calls the info function of each Edge in the list.
 */
typedef struct Edge_list{
	//Edge *edge = NULL;
	Edge *head = NULL;
	Edge *tail = NULL;

	void add_edge(Edge *e);
	void info();
	~Edge_list();
}Edge_list;

/**
 * \struct
 * This struct represents a polygon in the environment.
 * Its parameters are:
 * @param pl: point_list*. Is the list of points delimiting the polygon
 * perimeter.
 * @param centroid: point_node*. Is the point representing the centroid of the
 * polygon. It is computed averaging the coordinates of the points in the
 * perimeter.
 * @param pnext: polygon*. Is a pointer to another polygon istance, useful for
 * implementing a list.
 * The method available are:
 * @see edgify(): It is the function responsible to turn the polygon into a
 * list of edges.
 * @see add_offset(double offset): It is the function responsible to turn a
 * polygon into an enlarged version of itself.
 * @see concatenate(polygon a): It is the function allowing to concatenate
 * a polygon with another.
 * @see to_boost_polygon(): It is the function responsible to turn a polygon
 * into a boost::geometry::model::Polygon_boost object, ready to use by the boost
 * library.
 * @see info(): It is the function responsible to print all the information
 * representing a polygon and calls the info() functions of the point_node 
 * elements.
 */
typedef struct polygon{
	point_list *pl = NULL;
	point_node *centroid = NULL;
	polygon *pnext = NULL;
	
	// constructor
	polygon(point_list* pls);
	~polygon();

	// Methods
	Edge_list* edgify();
  	polygon* add_offset(double offset);  // Our polygon class
  	void concatenate(polygon *p);
	Polygon_boost to_boost_polygon();  // Polygon_boost of boost lybrary
  	void info();
}polygon;

double sinc(double);
double mod2pi(double);
double rangeSymm(double);
bool check(double, double, double, double, double, double, double, double);
Mat plot_points(point_list *, Mat, Scalar, bool, int thickness=1);
void sort(double_list *, point_list *);
tuple <double, double> get_new_point(double,double,double,double);

#endif

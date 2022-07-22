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

#include "../Log/logger.h"
#include "../../../simulator/src/9_project_interface/include/utils.hpp"
#include "../config.hpp"

#define SCALE_1 DIM_X_PLOT / 2
#define SCALE_2 DIM_Y_PLOT / 2

using pt = boost::geometry::model::d2::point_xy<double>;
using Polygon_boost = boost::geometry::model::polygon<pt>;
using Edge_boost = boost::geometry::model::segment<pt>;

using namespace cv;
using namespace std;

/**
 * \struct
 * It is the structure representing a double elment inside a list of float.
 * Its attributes are:
 * @param value: double. It is the value represented; default it 0;
 * @param pnaxt: \*double_node. It is the pointer to a new instance of the
 *                              structure. Necessary for a list.
 * The constructor takes as input a double value, default set to 0.
 */
struct double_node
{
	double value = 0;
	double_node *pnext = NULL;

	double_node(double a = 0);
} typedef double_node;

/**
 * \struct
 * This structure represent a list of doubles where each node is of type
 * double_node.
 * The attributes of the structure are:
 * @param head: \*double_node. It is the pointer to the head of the list.
 * @param tail: \*double_node. It is the pointer to the tail of the list.
 * @param size: int. It is the integer value representing the number of
 *                   elements contained in the list.
 *
 * Available methods are:
 * @see add_node: It is the method which allows to add a new element of type
 *                double_node to the list.
 * @see delete_list: It is the method allowing to delete from the list and from
 *                   the heap all the elements contained in the list.
 * @see print_list: It is the method allowing to print all the elemnts
 *                  contained in the list.
 */
struct double_list
{
	double_node *head = NULL;
	double_node *tail = NULL;
	int size = 0;

	void add_node(double_node *);
	void delete_list();
	void print_list();

} typedef double_list;

/**
 * \struct
 * This struct is used to represent a point in a 2D space in which its
 * position is represented by 2 double values.
 *
 * Available attributes are:
 * @param x: double. It is the position of the point with respect to the
 *           abscissa axis.
 * @param y: double. It is the position of the point with respect to the
 *           ordinate axis.
 * @param pnext: \*point_node. It is the pointer to another instance of type
 *               point_node necessary to build a list if needed.
 *
 * Available methods are:
 * @see point_node(double a, double b): It is the constructor of the struct.
 * @see copy(): It is the method used to return a copy of the struct.
 * @see Print(): It is the metho used to print the position of the point.
 * @see distance(point_node* p): It is the method used to compute the cartesian
 *                               distance between the struct and another
 *                               point_node instance.
 */
struct point_node
{
	double x = 0;
	double y = 0;
	point_node *pnext = NULL;

	point_node(double a = 0, double b = 0);
	point_node *copy();
	void Print();
	double distance(point_node *p = NULL);
	// ~point_node();
    bool operator == (const point_node &p);
    bool operator != (const point_node &p);
    pt to_boost();
} typedef point_node;

/**
 * \struct
 *  This struct represents a list of points made of point_node instances.
 *
 *  Available attributes are:
 *  @param head: point_node*. It is the pointer to the head of the list.
 *  @param tail: point_node*. It is the pointer to the tail of the list.
 *  @param x_min: double. It is the minimum value over the abscissa axis.
 *  @param y_min: double. It is the minimum value over the ordinate axis.
 *  @param x_max: double. It is the maximum value over the abscissa axis.
 *  @param y_max: double. It is the maximum value over the ordinate axis.
 *  @param size: int. It is the integer value representing how many elements
 *                    there are in the list.
 *
 *  Available methods are:
 *  @see add_node(point_node*, int iterations=1): It is the method allowing
 *       to add a point_node elements a number iterations of times to the list.
 *  @see append_list(point_list *e): It is the method allowing to add another
 *       list at the end of this list.
 *  @see print_list(): This method allows to print all the elements inside the
 *                     list.
 *  @see delete_list(): This method allows to remove all the elements from
 *                      the list and from the heap.
 *  @see pop(): This method allows to remove an element from the list.
 *  @see to_boost_polygon(): This method allows to convert the point_list
 *                           to a boost::geometry::model::polygon object.
 */
struct point_list
{
	point_node *head = NULL;
	point_node *tail = NULL;
	double x_min, y_min = 0;
	double x_max, y_max = 0;
	int size = 0;

	void add_node(point_node *, int iterations = 1);
	void append_list(point_list *e);
	void print_list();
	void delete_list();
	void pop();
	Polygon_boost to_boost_polygon();
	point_list *copy();
    point_list * orderify(int axis=0);
    bool is_in(point_node * p);
	// ~point_list();
} typedef point_list;

/**
 * \struct
 * Edge struct. Represents the edges composing a polygon.
 * Attributes:
 * @param points : pointer of type point_list. Contains two points of the
 * 				   edge.
 * @param next : pointer of type Edge. Points to the next instance.
 * @param slope : pointer of type double. The slope of the edge.
 *
 * Available methods are:
 * @see Edge(): Is is the struct costructor.
 * @see ~Edge(): It is the struct destructor.
 * @see intersection(Edge e): It checks whether the edge intersects with another.
 * @see middle_point(): It is the method return a point_node instance
 *                      representing the middle point_of the edge.
 * @see info(): It prints the coordinates of the points constituing an edge and
 * its slope.
 */
typedef struct Edge
{
	point_list *points;
	double *slope = new double;
	Edge *next = NULL;

	// Constructor
	Edge(point_node *p_1, point_node *p_2);

	// Methods
	point_node *intersection(Edge *e);
	point_node *middle_point();
	void info();
    Edge * copy();
	
    // Destructor
	~Edge();
} Edge;

/**
 * \struct
 * This struct implements a list of edges.
 * It is constituted by:
 * @param head: Edge*. It is the head of the list.
 * @param tail: Edge*. It is the tail of the list.
 *
 * The methods available are:
 * @see add_edge(Edge): allows to add an edge to the list.
 * @see info(): Calls the info function of each Edge in the list.
 * @see ~Edge_list(): It is the struct destructor.
 */
typedef struct Edge_list
{
	// Edge *edge = NULL;
	Edge *head = NULL;
	Edge *tail = NULL;
	int size = 0;

	void add_edge(Edge *e);
	void info();
	~Edge_list();
} Edge_list;

/**
 * \struct
 * This struct represents a polygon in the environment.
 *
 * Its parameters are:
 * @param id: string. It is the string representing the id of the polygon.
 *            Default to "NaN".
 * @param pl: point_list*. Is the list of points delimiting the polygon
 * perimeter.
 * @param centroid: point_node*. Is the point representing the centroid of the
 * polygon. It is computed averaging the coordinates of the points in the
 * perimeter.
 * @param pnext: polygon*. Is a pointer to another polygon istance, useful for
 * implementing a list.
 * @param area: double. It is the value representing the area of the polygon.
 * @param common_edges: map<string, Edge_list * >. List to know with which cells
 * has some edges in common.
 * 
 * The method available are:
 * @see polygon(point_list* pls): It is the struct constructor.
 * @see ~polygon(): It is the class destructor.
 * 
 * @see edgify(): It is the function responsible to turn the polygon into a
 * list of edges.
 * @see add_offset(double offset): It is the function responsible to turn a
 * polygon into an enlarged version of itself.
 * @see concatenate(polygon a): It is the function allowing to concatenate
 * a polygon with another.
 * @see recompute_centroid(): It is the method used to recompute the centroid
 *                            of the polygon if needed.
 * @see to_boost_polygon(): It is the function responsible to turn a polygon
 * into a boost::geometry::model::Polygon_boost object, ready to use by the boost
 * library.
 * @see points_in_common(polygon *p): It is the method used to check the number
 *                                    of points in common with another polygon.
 * @see info(): It is the function responsible to print all the information
 * representing a polygon and calls the info() functions of the point_node
 * elements.
 * @see copy(): Return a copy of the polygon.
 * @see add_common_edge(string s, Edge *e): Set a common edge between the polygon
 * and the cell with the given id.
 */
typedef struct polygon
{
	string id = "NaN";
	point_list *pl = NULL;
	point_node *centroid = NULL;
	polygon *pnext = NULL;
	double area = 0.0; // It is setted only if is a free space cells,
					   // look at roadmap.cpp -> \fun make_free_space_cells_squares
	map<string, Edge_list *> common_edges;

	// constructor
	polygon(point_list *pls, string _id = "NaN");
	~polygon();

	// Methods
	Edge_list *edgify();
	polygon *add_offset(double offset); // Our polygon class
	void concatenate(polygon *p);
	void recompute_centroid();
	Polygon_boost to_boost_polygon(); // Polygon_boost of boost library
	int points_in_common(polygon *p = NULL);
	void info();
	polygon *copy();
    point_list* intersections_with_edge(Edge* e);
    bool is_vertex(point_node *p);
	void add_common_edge(string s = "NaN", Edge *e = NULL);
} polygon;

/**
 * \struct list_of_polygons
 * It is the struct representing a list of polygon instances.
 *
 * Available attributes are:
 * @param head: polygon\*. It is the pointer to the head of the list.
 * @param tail: polygon\*. It is the pointer to the tail of the list.
 * @param size: int. It is the value representing how many elements are inside.
 *
 * Available methods are:
 * @see add_polygon(polygon* p): It is the method used to add a polygon to the
 *                               polygon list.
 * @see append_other_list(list_of_polygons * p): It is the method used to add
 *                                               another list at the end.
 */
typedef struct list_of_polygons
{
	polygon *head = NULL;
	polygon *tail = NULL;
	int size = 0;

	void add_polygon(polygon *p);
	void append_other_list(list_of_polygons *p = NULL);
} list_of_polygons;

/**
 * \struct
 * This struct represents a special case of list of polygons, the list of
 * obstacles.
 *
 * Available attributes are:
 * @param head: polygon\*. It is the pointer to the head of the list.
 * @param tail: polygon\*. It is the pointer to the tail of the list.
 * @param offset_head: polygon\*. It is the pointer to the head of the list of
 *                     offsetted polygons.
 * @param offset_tail: polygon\*. It is the pointer to the tail of the list of
 *                     offsetted polygons.
 * @param offset: double. It is the value representing how much the polygons
 *                        have to be enlarged.
 * @param size: int. It is the value representing how many normal polygons are
 *                   inside the list.
 * @param offset_size: int. It is the value representing how many offset
 *                          polygons are inside the list.
 *
 * Available methods are:
 * @see delete_offsetted_list(): It is the method used to remove the elements
 *                               contained in the offsetted list and in the
 *                               heap.
 * @see ~list_of_obstacle(): It is the struct destructor.
 */
typedef struct list_of_obstacles
{
	polygon *head = NULL;
	polygon *tail = NULL;
	polygon *offset_head = NULL;
	polygon *offset_tail = NULL;
	double offset = OFFSET; // 101e-3
	int size = 0;
	int offset_size = 0;

	void delete_offsetted_list();
	~list_of_obstacles();
} list_of_obstacles;

/**
 * \fun
 * This function implements a sinc function and returns its value applied to
 * the double given in input.
 */
double sinc(double);
double mod2pi(double);
double rangeSymm(double);
bool check(double, double, double, double, double, double, double, double);
/**
 * \fun plot_points
 * This function is used to plot a point list into a Mat object representing an
 * image.
 */
Mat plot_points(point_list *, Mat, Scalar, bool, int thickness = 1,
				bool show = false);

/**
 * \fun sort(double_list*, point_list*)
 * This function sort a point list according to the order of its associated doublelist
 * @param double_list *t
 * @param point_list *pts
 */
void sort(double_list *, point_list *);

/**
 * \fun get_new_point(double m1, double m2, double q1, double q2)
 * This function calculate the intersection point given the angular coifficents of two lines and their ordered at the origin.
 * @param double m1: Angular coifficient of the first line
 * @param double m2: Angular coifficient of the second line
 * @param double q1: Ordered at the origin of the first line
 * @param double q1: Ordered at the origin of the second line
 * @returns <double,double>: x,y coordinates of the intersection point
 */
tuple<double, double> get_new_point(double, double, double, double);

/**
 * \fun get_angle(double xc, double yx, double x, double y)
 * This function return the angle formed by the center of a circonference and a point that is part of it.
 * @param double xc: x-coordinate of the circonference's center
 * @param double yc: y-coordinate of the circonference's center
 * @param double x: x-coordinate of the circonference's point
 * @param double y: y-coordinate of the circonference's point
 * @return double: angle formed by the center and the point
 */
double get_angle(double, double, double, double);

/**
 * \fun is_in_arc(double start_angle, double end_angle, double angle)
 * This function is used to check if an angle is between two other angles.
 * @param double th0: Starting angle
 * @param double thf: Ending angle
 * @param double th: Angle to check
 * @return bool: boolean that says if the angle is included.
 */
bool is_in_arc(double, double, double);

/**
 * \fun boost_polygon_to_polygon(Polygo_boost p)
 * This function is used to transform a Boost polygon to a polygon instance.
 * @param Polygon_boost p: It is the boost polygon to transform.
 * @return polygon: the polygon instance representing the boost one.
 */
polygon *boost_polygon_to_polygon(Polygon_boost p, string _id = "NaN");

/**
 * \fun boost_polygon_to_point_list(Polygon_boost p)
 * This function is used to transform a Boost polygon to a point_list instance.
 * @param p: Polygon_boost. It is the Boost polygon to transform.
 * @return point_list: It is the point_list representing the Boost polygon.
 */
point_list *boost_polygon_to_point_list(Polygon_boost p);

/**
 * \fun cross2D(double *v1, double *v2)
 *
 * @return double
 */
double cross2D(double *, double *);

/**
 * \fun dot2D(double *v1, double *v2)
 *
 * @return double
 */
double dot2D(double *, double *);

/**
 * \fun tuple<point_list *, double_list *> intersLineLine(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
 * This function return the intersection point's list of two lines.
 * @param double x1: x-coordinate of the first point of the first line
 * @param double y1: y-coordinate of the first point of the first line
 * @param double x2: x-coordinate of the second point of the first line
 * @param double y2: y-coordinate of the seconf point of the first line
 * @param double x3: x-coordinate of the first point of the second line
 * @param double y3: y-coordinate of the first point of the second line
 * @param double x4: x-coordinate of the second point of the second line
 * @param double y4: y-coordinate of the second point of the second line
 * @return tuple<point_list *, double_list *>
 */
// tuple<point_list *, double_list *> intersLineLine(double, double, double, double, double, double, double, double);

bool intersLineLine(point_node *, point_node *, point_node *, point_node *);

/**
 * \fun tuple<point_list *, double_list *> intersCircleLine(double a, double b, double r, double x1, double y1, double x2, double y2)
 * This function return the intersection points between a circle and a line.
 * @param double a: x-coordinate of the center of the circle
 * @param double b: y-coordinate of the center of the circle
 * @param double r: radius of the circle
 * @param double x1: x-coordinate of the first point of the line
 * @param double y1: y-coordinate of the first point of the line
 * @param double x2: x-coordinate of the second point of the line
 * @param double y2: y-coordinate of the second point of the line
 * @returns tuple<point_list *, double_list *>
 */
tuple<point_list *, double_list *> intersCircleLine(double, double, double, double, double, double, double);

/**
 * \fun point_node *los(point_node *p1, point_node *p2, polugon *pol, Edge *common_edge)
 * This function check the line formed between two centroids intersects with any line in the arena.
 * If yes, the two points are not in line of sight.
 * 
 * @param p1: point_node *. First centroid.
 * @param p2: point_node *. Second centroid.
 * @param pol: First element of the obstacles to check. 
 * @param common_edge: Common edge between the 2 cells having p1 and p2 as centroids.
 * @return point_node*: Null if in los. Mid-point of the common edge otherwise. 
 */
bool los(point_node *p1 = NULL, point_node *p2 = NULL, polygon *pol = NULL, vector<Edge*> *sweep_line = NULL);

/**
 * \fun polygon *merge(polygon *p1, polygon *p2)
 * This function merge two given polygons.
 * 
 * @param p1: polygon *. First polygon
 * @param p2: polygon *. Second polygon
 * @return polygon *: Merged polygon 
 */
polygon *merge(polygon *p1, polygon *p2);

/**
 * \fun Edge *find_common_edge(polygon *p1, polygon *p2)
 * This function find the common edge between two given polygons.
 * @param p1: polygon. First polygon 
 * @param p2: polygon. Second polygon 
 * @return Edge *: Common Edge. 
 */
Edge *find_common_edge(polygon *p1 = NULL, polygon *p2 = NULL);


list_of_polygons *subset_over_middle_point(polygon *p = NULL);

/**
 * \fun difference_of_vectors(vector<Polygon_boost> arena, vector<Polygon_boost> obstacles)
 * This function subtracts a vector of polygons from another one.
 * @param arena: vector<Polygon_boost>. Is the vector of polygons from which
 * the other polygons will be subtracted.
 * @param obstacles: vector<Polygon_boost>. Is the vector of polygons that will
 * be subtracted. 
 * @return vector<Polygon_boost>: subtracted vector. 
 */
vector<Polygon_boost> difference_of_vectors(vector<Polygon_boost> arena,
											vector<Polygon_boost> obstacles);

string PDDL_conditional_cost(string el1, string el2, double cost,
                             string cost_name="total-cost");
bool compare_doubles(double a, double b);
bool equal_point_nodes(point_node * a, point_node * b);
bool point_belong_to_edge(point_node *p, Edge * e);
point_node * edge_has_vertex(Edge * e, polygon * pol);
bool point_less_edge(Edge * e, point_node * p);
point_list * order_pair_ascending(point_node * a, point_node* b, int axis=0);
#endif

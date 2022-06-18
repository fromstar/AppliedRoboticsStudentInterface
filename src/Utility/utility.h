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
#include "../../../simulator/src/9_project_interface/include/utils.hpp"


#define DIM_X_PLOT 600
#define DIM_Y_PLOT 900

#define SCALE_1 DIM_X_PLOT/2
#define SCALE_2 DIM_Y_PLOT/2

using pt = boost::geometry::model::d2::point_xy<double>;
using Polygon_boost = boost::geometry::model::polygon<pt>;


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
struct double_node{
	double value = 0;
	double_node *pnext = NULL;
	
	double_node(double a=0);
}typedef double_node;


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
struct double_list{
	double_node *head = NULL;
	double_node *tail = NULL;
	int size = 0;
	
	void add_node(double_node *);
	void delete_list();
	void print_list();
	
}typedef double_list;

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
struct point_node{
	double x = 0;
	double y = 0;
	point_node *pnext = NULL;
	
	point_node(double a=0, double b=0);
	point_node* copy();
	void Print();
	double distance(point_node* p=NULL);
}typedef point_node;

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
struct point_list{
	point_node *head = NULL;
	point_node *tail = NULL;
	double x_min, y_min;
	double x_max, y_max;
	int size = 0;
	
	void add_node(point_node *, int iterations=1);
	void append_list(point_list *e);
	void print_list();
	void delete_list();
	void pop();
    Polygon_boost to_boost_polygon();
}typedef point_list;

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
 * @see info(): It prints the coordinates of the points constituing an edge and
 * its slope.
 * @see middle_point(): It is the method return a point_node instance
 *                      representing the middle point_of the edge.
 * @see intersection(Edge e): It checks whether the edge intersects with another.
 * @see ~Edge(): It is the struct destructor.
 */
typedef struct Edge{
	point_list* points;
	double *slope = new double;
	Edge *next = NULL;

	// Constructor
	Edge(point_node *p_1, point_node *p_2);

	// Destructor
    ~Edge();

	// Methods
	point_node* intersection(Edge *e);
	point_node* middle_point();
	void info();
}Edge;

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
typedef struct Edge_list{
	//Edge *edge = NULL;
	Edge *head = NULL;
	Edge *tail = NULL;
	int size = 0;

	void add_edge(Edge *e);
	void info();
	~Edge_list();
}Edge_list;

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
 *
 * The method available are:
 * @see polygon(point_list* pls): It is the struct constructor.
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
 * @see ~polygon(): It is the class destructor.
 */
typedef struct polygon{
    string id = "NaN";
	point_list *pl = NULL;
	point_node *centroid = NULL;
	polygon *pnext = NULL;
    double area = 0.0;  // It is setted only if is a free space cells,
                        // look at roadmap.cpp -> \fun make_free_space_cells_squares

	// constructor
	polygon(point_list* pls);
	~polygon();

	// Methods
	Edge_list* edgify();
  	polygon* add_offset(double offset);  // Our polygon class
  	void concatenate(polygon *p);
	void recompute_centroid();
	Polygon_boost to_boost_polygon();  // Polygon_boost of boost library
    int points_in_common(polygon *p);
  	void info();
}polygon;

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
	void append_other_list(list_of_polygons *p);
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
	double offset = 101e-3;
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
Mat plot_points(point_list *, Mat, Scalar, bool, int thickness=1,
				bool show=false);

/**
 * \fun sort(double_list*, point_list*)
 *
 */
void sort(double_list *, point_list *);

/**
 * \fun get_new_point
 */
tuple <double, double> get_new_point(double,double,double,double);

/**
 * \fun exec
 */
string exec(const char *cmd);

/**
 * \fun get_angle
 */
double get_angle(double,double,double,double);

/**
 * \fun is_in_arc
 *
 */
bool is_in_arc(double,double,double);

/**
 * \fun boost_polygon_to_polygon(Polygo_boost p)
 * This function is used to transform a Boost polygon to a polygon instance.
 * @param Polygon_boost p: It is the boost polygon to transform.
 * @return polygon: the polygon instance representing the boost one.
 */
polygon *boost_polygon_to_polygon(Polygon_boost p);

/**
 * \fun boost_polygon_to_point_list(Polygon_boost p)
 * This function is used to transform a Boost polygon to a point_list instance.
 * @param p: Polygon_boost. It is the Boost polygon to transform.
 * @return point_list: It is the point_list representing the Boost polygon.
 */
point_list *boost_polygon_to_point_list(Polygon_boost p);
#endif

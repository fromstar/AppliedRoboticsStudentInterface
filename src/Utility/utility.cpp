#include "utility.h"

/**
 * \fun double_node(double a)
 * It is the constructor of the struct double_node.
 * @param a: double. It is the value represented by the structure.
 */
double_node::double_node(double a)
{
    value = a;
}

/**
 * \fun add_node(double_node *node)
 * It is the method allowing the insertion of a new double_node inside
 * the double_list structure.
 * @param node: \*double_node. It is the pointer to a double_node instance.
 */
void double_list::add_node(double_node *node)
{
	size++;
	if (head == NULL)
	{
		head = node;
		tail = head;
		return;
	}
	tail->pnext = node;
	tail = tail->pnext;
}

/**
 * \fun print_list()
 * This method it is used to print the content of a double_list.
 */
void double_list::print_list()
{
	double_node *tmp = head;
	while (tmp != NULL)
	{
		cout << "Value: " << tmp->value << endl;
		tmp = tmp->pnext;
	}
}

/**
 * \fun delete_list()
 * This method is used to remove all the elements from the double_list
 * structure and also from the heap.
 */
void double_list::delete_list()
{
	double_node *tmp;
	while (head != NULL)
	{
		tmp = head;
		head = head->pnext;
		delete tmp;
	}
	size = 0;
}


/**
 * \fun point_node(double a, double b)
 * It is the constructor of the point_node struct.
 * @param a: double. It is the point position wrt the abscissa axis.
 * @param b: double. It is the point positizion wrt the ordinate axis.
 */
point_node::point_node(double a, double b)
{
    x = a;
    y = b;
}

/**
 * \fun copy()
 * This method is used to return a copy instance of the struct.
 * @return pointer of type point_node to the copy instance.
 */
point_node *point_node::copy()
{
	return new point_node(x, y);
};

/**
 * \fun Print()
 * This method is used to print the positio over the coordinates system
 * of the point represented.
 */
void point_node::Print()
{
	printf("%0.4f,%0.4f\n", x, y);
};

/**
 * \fun distance(point_node* p)
 * This method is used to compute the distance between the point
 * represented and another instance of type point_node*.
 * 
 * @param p: point_node\*. It is the instance of point_node to which compute
 *                         the distance.
 *
 * @return double: It is the euclidean distance among the point_node instances.
 */
double point_node::distance(point_node* p)
{
    if (p == NULL)
    {
        cout << "Given point is NULL" << endl;
    }
    return sqrt(pow(x - p->x, 2) + pow(y - p->y, 2));
};

/**
 * Function to add a point in a list of points.
 * @param node: poiter of type point_node.
 * @param iterations: int.
 * 					 Specifies the number of times the point has to be added.
 * 					 This reduces a bit the code.
 */
void point_list::add_node(point_node *node, int iterations)
{
	for (int i = 0; i < iterations; i++)
	{
		size++;
		if (head == NULL)
		{
			head = node;
			tail = head;
			x_min = node->x;
			x_max = x_min;
			y_min = node->y;
			y_max = y_min;
			return;
		}

		x_min = std::min(x_min, node->x);
		x_max = std::max(x_max, node->x);
		y_min = std::min(y_min, node->y);
		y_max = std::max(y_max, node->y);

		tail->pnext = node;
		tail = tail->pnext;
	};
}

/**
 * \fun pop()
 * This method removes the last element from the list.
 * @return point_list*: pointer to the istance of point_list without the last
 *                      element.
 */
void point_list::pop(){
	point_node *tmp = head;
	while(tmp->pnext->pnext != NULL){
		tmp = tmp->pnext;
	};
    delete tmp->pnext;
    tmp->pnext = NULL;
    tail = tmp;
};

/**
 * \fun append_list(point_list *e)
 * This method allows to add another list of type point_list to the
 * one represented by the structure.
 *
 * @param e: point_list\*. It is the pointer to the head of the point list
 *           to append.
 */
void point_list::append_list(point_list *e)
{
	tail->pnext = e->head;
	tail = e->tail;
};

/**
 * \fun print_list()
 * This method is used to print all the elements in the list.
 */
void point_list::print_list()
{
	point_node *tmp = head;
	int i = 0;
	while (tmp != NULL)
	{
		i++;
		cout << "Point: " << i << endl;
		cout << "x: " << tmp->x << "\ny: " << tmp->y << endl
			 << endl;
		tmp = tmp->pnext;
	}
}

/**
 * \fun delete_list()
 * This method is used to remove all the elements from the list and also from
 * the heap.
 */
void point_list::delete_list()
{
	point_node *tmp;
	while (head != NULL)
	{
		tmp = head;
		head = head->pnext;
		delete tmp;
	}
	size = 0;
}

/**
 * \fun Edge(point_node* p_1, point_node* p_2)
 * This is the constructor for the Edge struct.
 * @param p_1: point_node\*. Pointer representing the first point of the edge.
 * @param p_2: point_node\*. Pointer representing the end point of the edge.
 */
Edge::Edge(point_node *p_1, point_node *p_2)
{
	points = new point_list;
	points->add_node(new point_node(p_1->x, p_1->y));
	points->add_node(new point_node(p_2->x, p_2->y));
	*slope = (p_2->y - p_1->y) / (p_2->x - p_1->x);
};

/**
 * \fun
 * This method implements the destructor of the struct.
 */
Edge::~Edge()
{
	points->delete_list();
};

/**
 * \fun intersection(Edge* e)
 * This method is used to compute the intersection with another edge.
 * @param e: Edge*. It is the edge instance with which an intersection has
 *                  to be found if any.
 * @return point_node* the point_node representing the intersection. It returns
 *         NULL if no intersection is found.
 */
point_node *Edge::intersection(Edge *e)
{
	// given the slope, the equation of a line is y-y1 = m(x-x1).
	if (*slope == *e->slope)
	{
		return NULL;
	};
	point_node *start = points->head;
	point_node *end = points->tail;
	point_node *e_start = e->points->head;
	point_node *e_end = e->points->tail;

	double x_int = (e_start->y - start->y + (start->x * *slope) -
					(e_start->x * *e->slope)) /
				   (*slope - *e->slope);
	double y_int = (*slope * (x_int - start->x) + start->y);

	// Check intersection point to avoid that relies over the segments
	if (x_int < min(points->x_min, e->points->x_min) ||
		x_int > max(points->x_max, e->points->x_max) ||
		y_int < min(points->y_min, e->points->y_min) ||
		y_int > max(points->y_max, e->points->y_max))
	{
		return NULL;
	};

	point_node *intersection = new point_node(x_int, y_int);
	return intersection;
};

/**
 * \fun middle_point()
 * This method computes the middle point of an edge instance.
 * @return point_node*: It is the pointer to the point_node instance
 *                      representing the middle_point.
 */
point_node* Edge::middle_point(){
	double mid_x = ((points->head->x + points->tail->x))/2;
	double mid_y = ((points->head->y + points->tail->y))/2;
	return new point_node(mid_x, mid_y);
};


/**
 * \fun info()
 * This method is used to print informations regarding the edge, like the
 * positions of its points and its slope.
 */
void Edge::info()
{
	points->print_list();
	printf("Slope: %f\n", *slope);
};

/**
 * \fun add_edge(Edge* e)
 * This method allows to add a new Edge instance to the list of edges.
 * @param e: Edge\*. It is the pointer to the Edge instance to add to the list.
 */
void Edge_list::add_edge(Edge *e)
{
	if (head == NULL)
	{
		head = e;
		tail = head;
		size++;
		return;
	};

	tail->next = e;
	tail = tail->next;
	size++;
};

/**
 * \fun info()
 * This method is used to print all the elements contained in the list.
 */
void Edge_list::info()
{
	Edge *tmp = head;
	while (tmp != NULL)
	{
		tmp->info();
		tmp = tmp->next;
	};
};

/**
 * \un ~Edge_list()
 * It is the struct destructor.
 */
Edge_list::~Edge_list()
{
	Edge *tmp = NULL;
	while (head != NULL)
	{
		tmp = head;
		head = head->next;
		delete tmp;
	};
};

/**
 * \fun plot_points
 * This function is used to plot a point list to a Mat object representing
 * an image.
 *
 * @param pl: point_list*. It is the list of points to plot.
 * @param arena: Mat. It is the Mat object in which the points will be plotted.
 * @param colorline: cv::Scalar. It is the color in which the point list will
 *                   be drown. It is expressed in RGB.
 * @param isPolygon: bool. It is a flag telling whether or not the point list
 *                   that is being plotted represents a polygon.
 * @param thickness: int. It is an integer value representing how thick will
 *                   be the line drown.
 * @param show: bool. It is a flag telling whether or not to print details
 *              of the line plotted.
 */
Mat plot_points(point_list *pl, Mat arena, Scalar colorline, bool isPolygon,
				int thickness, bool show)
{
	if (pl != NULL)
	{
		point_node *n1 = pl->head;
		point_node *n2;
		while (n1 != NULL && n1->pnext != NULL)
		{
			n2 = n1->pnext;
			cv::Point pt1((n1->x * SCALE_1) + SCALE_2, (n1->y * -SCALE_1) + SCALE_2);
			cv::Point pt2((n2->x * SCALE_1) + SCALE_2, (n2->y * -SCALE_1) + SCALE_2);
			line(arena, pt1, pt2, colorline, thickness);
			if (show){
				cout << "Plotting Line: "
					 << "[" << (n1->x * SCALE_1) + SCALE_2 << ":"
					 << (n1->y * -SCALE_1) + SCALE_2 << "]\t"
					 << "[" << (n2->x * SCALE_1) + SCALE_2 << ":"
					 << ((n2->y * -SCALE_1) + SCALE_2) << "]" << endl;
			};
			n1 = n1->pnext;
		}

		if (isPolygon){ // close the polygon connecting the last point with the firts one
			line(arena, cv::Point((pl->tail->x * SCALE_1) + SCALE_2,
							  (pl->tail->y * -SCALE_1) + SCALE_2),
				 cv::Point((pl->head->x * SCALE_1) + SCALE_2,
					   (pl->head->y * -SCALE_1) + SCALE_2),
				 colorline, thickness);
			if (show){
				cout << "Plotting Line: "
					 << "[" << (n1->x * SCALE_1) + SCALE_2 << ":"
					 << (n1->y * -SCALE_1) + SCALE_2 << "]\t"
					 << "[" << (pl->head->x * SCALE_1) + SCALE_2 << ":"
					 << ((pl->head->y * -SCALE_1) + SCALE_2) << "]" << endl;
			};
		};
				
	}
	return arena;
}

/**
 * \fun
 * This function implements the sinc operator.
 * 
 * @param t 
 * @return double 
 */
double sinc(double t)
{
	if (abs(t) < 0.002)
		return 1 - pow(t, 2) / 6 * (1 - pow(t, 2) / 20);
	else
		return sin(t) / t;
}

/**
 * \fun
 * This function implements the mod2pi operator.
 * 
 * @param ang 
 * @return double 
 */
double mod2pi(double ang)
{
	double out = ang;
	while (out < 0)
		out = out + 2 * M_PI;
	while (out >= 2 * M_PI)
		out = out - 2 * M_PI;
	return out;
}

double rangeSymm(double ang)
{
	double out = ang;
	while (out <= -M_PI)
		out = out + 2 * M_PI;
	while (out > M_PI)
		out = out - 2 * M_PI;
	return out;
}

bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf)
{
	double x0 = -1;
	double y0 = 0;
	double xf = 1;
	double yf = 0;
	double eq1, eq2, eq3;
	double Lpos;
	eq1 = x0 + s1 * sinc((1 / 2.) * k0 * s1) * cos(th0 + (1 / 2.) * k0 * s1) + s2 * sinc((1 / 2.) * k1 * s2) * cos(th0 + k0 * s1 + (1 / 2.) * k1 * s2) + s3 * sinc((1 / 2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1 / 2.) * k2 * s3) - xf;
	eq2 = y0 + s1 * sinc((1 / 2.) * k0 * s1) * sin(th0 + (1 / 2.) * k0 * s1) + s2 * sinc((1 / 2.) * k1 * s2) * sin(th0 + k0 * s1 + (1 / 2.) * k1 * s2) + s3 * sinc((1 / 2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1 / 2.) * k2 * s3) - yf;
	eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);
	Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);
	return (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-10) && Lpos;
}


/**
 * \fun
 * This function sort a point list according to the order of its associated doublelist
 * 
 * @param t double_list *.
 * @param pts point_list *.
 * 
 */
void sort(double_list *t, point_list *pts)
{
	double_node *dbfst = t->head;
	double_node *dbsnd;
	point_node *ptfst = pts->head;
	point_node *ptsnd;
	while (dbfst != NULL)
	{
		dbsnd = dbfst->pnext;
		ptsnd = ptfst->pnext;
		while (dbsnd != NULL)
		{
			if (dbfst->value > dbsnd->value)
			{
				double tmp = dbfst->value;
				dbfst->value = dbsnd->value;
				dbsnd->value = tmp;

				tmp = ptfst->x;
				ptfst->x = ptsnd->x;
				ptsnd->x = tmp;

				tmp = ptfst->y;
				ptfst->y = ptsnd->y;
				ptsnd->y = tmp;
			}
			dbsnd = dbsnd->pnext;
			ptsnd = ptsnd->pnext;
		}
		dbfst = dbfst->pnext;
		ptfst = ptfst->pnext;
	}
}

/**
 * \fun
 * This function calculate the intersection point given the angular coifficents of two lines and their ordered at the origin.
 * Every possible case has been considered. That is, if a line is horizontal, vertical or perpendicular etc.
 * 
 * @param m1: double. Angular coifficient of the first line
 * @param m2: double. Angular coifficient of the second line
 * @param q1: double. Ordered at the origin of the first line
 * @param q2: double. Ordered at the origin of the second line
 * @return tuple<double, double>: Coordinates of the intersection point. 
 */
tuple<double, double> get_new_point(double m1, double m2, double q1, double q2)
{
	double x, y;

	/* First line horizontal, second line vertical.*/
	if (m1 == 0 && m2 == std::numeric_limits<double>::infinity())
	{
		x = q2;
		y = q1;
	}

	/* Second line horizontal, second line vertical*/
	else if (m2 == 0 && m1 == std::numeric_limits<double>::infinity())
	{
		x = q1;
		y = q2;
	}
	
	/* Just the first line horizontal*/
	else if (m1 == 0)
	{
		// y = q1
		// y = m2x + q2 ->  (y - q2)/m2 = x
		y = q1;
		x = (y - q2) / m2;
	}

	/* Just the second line horizontal*/
	else if (m2 == 0)
	{
		y = q2;
		x = (y - q1) / m1;
	}

	/* Just the first line vertical*/
	else if (m1 == std::numeric_limits<double>::infinity())
	{
		x = q1;
		y = m2 * x + q2;
	}

	/* Just the second line vertical*/
	else if (m2 == std::numeric_limits<double>::infinity())
	{
		x = q2;
		y = m1 * x + q1;
	}

	/* No lines horizontal or vertical*/
	else
	{
		x = (q1 - q2) / (m2 - m1);
		y = m1 * ((q1 - q2) / (m2 - m1)) + q1;
	}

	return make_tuple(x, y);
}

/**
 * \fun
 * Constructor of a new polygon::polygon object
 * 
 * @param pls: point_list *. Point's list of the polygon
 */
polygon::polygon(point_list *pls)
{
	if (pls->size >= 3)
	{
		pl = pls;
        pt new_centroid;
        boost::geometry::centroid(pls->to_boost_polygon(), new_centroid);
        centroid = new point_node(new_centroid.x(), new_centroid.y());
        area = boost::geometry::area(pls->to_boost_polygon());
	}
	else
	{
		throw std::invalid_argument("Error: A polygon can't have less"
									"than 3 points.\n");
		exit(-1);
	}
};

void polygon::recompute_centroid(){
	point_node *tmp = pl->head;
	double x_sum=0;
	double y_sum=0;
	while(tmp != NULL){
		x_sum += tmp->x;
		y_sum += tmp->y;
		tmp = tmp->pnext;
	};
	centroid =  new point_node(x_sum/pl->size, y_sum/pl->size);
};

/**
 * \fun
 * Destructor of the polygon
 */
polygon::~polygon()
{
	pl->delete_list();
	delete(centroid);
};

Edge_list *polygon::edgify()
{
	point_node *tmp = pl->head;
	Edge_list *edges = new (Edge_list);
	while (tmp->pnext != NULL)
	{
		Edge *e_temp = new Edge(tmp, tmp->pnext);
		edges->add_edge(e_temp);
		tmp = tmp->pnext;
	};
	Edge *last_edge = new Edge(tmp, pl->head);
	edges->add_edge(last_edge);
	return edges;
};

/**
 * \fun.
 * This function turn a polygon into an enlarged version of itself.
 * To do this a parallel line at distant offset is computed for each edge of a polygon.
 * Then the vertices are obtained by looking for the intersection point of the two lines 
 * associated with consecutive edges
 * 
 * @param offset: Distance of the lines. 
 * @return polygon* 
 */
polygon *polygon::add_offset(double offset)
{

	/* The idea is to calculate the lines coefficients in order to have the
	 * generic equations for them and then find where they intersecate */

	point_list *new_pol = new point_list;
	point_node *p1 = pl->head;
	point_node *p2 = p1->pnext;

	// A matrix that contains in x the angular coefficient and in y the
	// ordinates value of the lines offsetted

	double coeff[pl->size][2];

	double x, y;
	double center_x = 0, center_y = 0;

	while (p1 != NULL)
	{
		center_x += p1->x;
		center_y += p1->y;
		p1 = p1->pnext;
	}
	center_x = center_x / pl->size;
	center_y = center_y / pl->size;
	p1 = pl->head;

	// I have to find the equation for the parallel offsetted line
	for (int i = 0; i < pl->size; i++)
	{
		double x1 = p1->x;
		double y1 = p1->y;

		double x2 = p2->x;
		double y2 = p2->y;

		// middle point coordinates
		double xm = (x2 + x1) / 2;
		double ym = (y2 + y1) / 2;

		double m, q;
		double qp1, qp2;

		double d1;
		double d2;
		// Particular cases
		if (x2 == x1)
		{
			m = std::numeric_limits<double>::infinity();
			// If m=inf the line is vertical -> x = k
			qp1 = xm + offset;
			qp2 = xm - offset;

			d1 = abs(qp1 - center_x);
			d2 = abs(qp2 - center_x);
		}
		else if (y2 == y1)
		{
			m = 0;
			// If m=0-> y = k. In this case k = y +- offset
			qp1 = ym + offset;
			qp2 = ym - offset;
			d1 = abs(qp1 - center_y);
			d2 = abs(qp2 - center_y);
		}
		// Generic case
		else
		{
			// https://www.youmath.it/formulari/formulari-di-geometria-analitica/430-formule-retta.html
			// Angular coefficient of the two lines
			m = (y2 - y1) / (x2 - x1);
			// Ordered at the origin
			q = ((x2 * y1) - (x1 * y2)) / (x2 - x1);

			// The parallel line has same m but different q.
			// Distance between two lines: d = |q1 - q2|/sqrt(m+1)
			// with q2 unknow. So i have 2 possible solutions for q2
			qp1 = y1 - m * x1 - offset * sqrt(1 + pow(m, 2));
			qp2 = offset * sqrt(pow(m, 2) + 1) + y1 - m * x1;

			d1 = abs(center_y - (m * center_x + qp1)) / sqrt(1 + pow(m, 2));
			d2 = abs(center_y - (m * center_x + qp2)) / sqrt(1 + pow(m, 2));
		}
		coeff[i][0] = m;
		// If the distance from the center of gravity of the firs lines is
		// bigger than the second I have to take it.

		if (d1 > d2)
		{
			coeff[i][1] = qp1;
		}
		else
		{
			coeff[i][1] = qp2;
		}

		if (p2->pnext == NULL)
			p2 = pl->head;
		else
			p2 = p2->pnext;

		p1 = p1->pnext;
	}
	// The interesection coordinates of two lines are:
	// x = (q1-q2)/(m2-m1); y = m1 * ((q1 - q2) / (m2 - m1)) + q1

	for (int i = 1; i < pl->size; i++)
	{
		double m1 = coeff[i - 1][0];
		double m2 = coeff[i][0];
		double q1 = coeff[i - 1][1];
		double q2 = coeff[i][1];

		tie(x, y) = get_new_point(m1, m2, q1, q2);
		new_pol->add_node(new point_node(x, y));
	}
	tie(x, y) = get_new_point(coeff[pl->size - 1][0], coeff[0][0], coeff[pl->size - 1][1], coeff[0][1]);
	new_pol->add_node(new point_node(x, y));

	return new polygon(new_pol);
};

/**
 * \fun
 * This function create a boost::polygon with a point_list.
 * 
 * @return Polygon_boost 
 */
Polygon_boost point_list::to_boost_polygon()
{
	point_node *pn = head;
	string pts = "POLYGON((";
	while (pn != NULL)
	{
		pts.append(to_string(pn->x));
		pts.append(" ");
		pts.append(to_string(pn->y));
		pts.append(",");

		pn = pn->pnext;
	}
	pts.append("))");

	Polygon_boost p;
	boost::geometry::read_wkt(pts, p);
	if (!boost::geometry::is_valid(p))
	{
		boost::geometry::correct(p); // Fixes edge order -> e.g. clockwise
	};
    return p;
}

/**
 * \fun
 * This function converts a polygon into a boost::polygon
 * 
 * @return Polygon_boost 
 */
Polygon_boost polygon::to_boost_polygon()
{
	return pl->to_boost_polygon();
};

void polygon::concatenate(polygon *p)
{
	if (pl == NULL)
	{
		pl = p->pl;
	}
	else
	{
		pnext = p;
	};
};

int polygon::points_in_common(polygon *p)
{
    int points_in_common = 0;
    double epsilon = 1e-6;
    point_node *tmp_pol_1 = pl->head;    
    while(tmp_pol_1 != NULL)       
        {                                                       
            point_node *tmp_pol_2 = p->pl->head;
            while(tmp_pol_2 != NULL) // && points_in_common <= 1)   
            {   
                double eps_mod_x = max(fabs(tmp_pol_1->x), fabs(tmp_pol_2->x));
                double eps_mod_y = max(fabs(tmp_pol_1->y), fabs(tmp_pol_2->y));

                if (tmp_pol_1->x - tmp_pol_2->x <= epsilon * eps_mod_x &&         
                    tmp_pol_1->y - tmp_pol_2->y <= epsilon * eps_mod_y)
                {                                           
                    points_in_common += 1;                  
                };                                          
                tmp_pol_2 = tmp_pol_2 -> pnext;                 
             };                                                  
             tmp_pol_1 = tmp_pol_1 -> pnext;                     
         };                                                      
    return points_in_common;                             
}

void polygon::info()
{
	pl->print_list();
	printf("Polygon centroid: ");
	centroid->Print();
};

/**
 * \fun
 * This function add a polygon in the list of polygons of an arena.
 * 
 * @param p 
 */
void list_of_polygons::add_polygon(polygon *p)
{
	if (head == NULL)
	{
		head = p;
		tail = head;
		return;
	}
	tail->pnext = p;
	tail = tail->pnext;
	size += 1;
};

void list_of_polygons::append_other_list(list_of_polygons *p)
{
	if (head == NULL)
	{
		head = p->head;
		tail = p->tail;
	}
	else
	{
		tail->pnext = p->head;
		tail = p->tail;
	};
	size += p->size;
};

/**
 * \fun
 * Destroy the list of obstacles::list of obstacles object
 * 
 */
list_of_obstacles::~list_of_obstacles()
{
	polygon *tmp = head;
	polygon *otmp = offset_head;
	while (head != NULL)
	{
		tmp = head;
		head = head->pnext;
		delete tmp;
	}
	while (offset_head != NULL)
	{
		tmp = offset_head;
		offset_head = offset_head->pnext;
		delete tmp;
	}
};

/**
 * \fun
 * Delete all the offsetted polygons.
 * 
 */
void list_of_obstacles::delete_offsetted_list()
{
	polygon *tmp;
	while (offset_head != NULL)
	{
		tmp = offset_head;
		offset_head = offset_head->pnext;
		delete tmp;
	}
	offset_size = 0;
};

string exec(const char *cmd)
{
	std::array<char, 128> buffer;
	std::string result;
	std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
	if (!pipe)
	{
		throw std::runtime_error("popen() failed!");
	}
	while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
	{
		result += buffer.data();
	}
	return result;
}

/**
 * \fun 
 * This function return the angle formed by a point into a circonference.
 * 
 * @param xc: double. x-coordinate of the circle's center.
 * @param yc: double. y-coordinate of the circle's center.
 * @param x: double. x-coordinate of the point.
 * @param y: double. y-coordinate of the point.
 * @return double 
 */
double get_angle(double xc, double yc, double x, double y)
{
	return atan2(y-yc, x-xc);
}

/**
 * \fun
 * This function check if angle is between a starting angle and a final angle
 * 
 * @param th0: double. Starting angle
 * @param thf: double. Finale angle
 * @param th: double. Angle to check
 * @return bool. True if angle is included. False otherwise
 */
bool is_in_arc(double th0, double thf, double th)
{
	if(th < th0 && th > thf)
	{
		return false;
	}
	return true;
}
/**
 * \fun
 * This function is used to convert a boost polygon object into a list of
 * points ready to be interpreted by others functions in the framework.
 * The only purpose of this function is to reduce the amount of code written.
 * @param[in] p: boost::geometry::model::Polygon_boost. Is the polygon to convert.
 * @parma[out] pl: point_list pointer. Is the resulting point list of the
 * polygon.
*/
point_list *boost_polygon_to_point_list(Polygon_boost p)
{
    point_list *pl = new point_list();                                     
    for (auto it = boost::begin(boost::geometry::exterior_ring(p));        
         it != boost::end(boost::geometry::exterior_ring(p)); ++it)        
    {                                                                      
            double x = boost::geometry::get<0>(*it);                                        
            double y = boost::geometry::get<1>(*it);                                        
                                                                               
            pl->add_node(new point_node(x, y));                                
        }                                                                      
    pl->pop();
    return pl;
}

/**
 * \fun
 * This function is used to convert a boost::polygon into a polygon.
 * 
 * @param p: Polygon_boost.
 * @return polygon* 
 */
polygon *boost_polygon_to_polygon(Polygon_boost p)                         
{   
    point_list * new_pol_list = boost_polygon_to_point_list(p);
    
    pt new_centroid;
    boost::geometry::centroid(p, new_centroid);

    polygon *new_pol = new polygon(new_pol_list); 
    new_pol->centroid = new point_node(new_centroid.x(), new_centroid.y());
    new_pol->area = boost::geometry::area(p);
    return new_pol;                                                        
};                                                                         


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
double point_node::distance(point_node *p)
{
	if (p == NULL)
	{
		cout << "Given point is NULL" << endl;
	}
	return sqrt(pow(x - p->x, 2) + pow(y - p->y, 2));
};

bool point_node::operator==(const point_node &p)
{
	if (compare_doubles(x, p.x) && compare_doubles(y, p.y))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool point_node::operator!=(const point_node &p)
{
	if (!compare_doubles(x, p.x) && !compare_doubles(y, p.y))
	{
		return true;
	}
	else
	{
		return false;
	}
}

pt point_node::to_boost()
{
	return pt(x, y);
}

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
			head = node->copy();
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

		tail->pnext = node->copy();
		tail = tail->pnext;
	};
}

/**
 * \fun pop()
 * This method removes the last element from the list.
 * @return point_list*: pointer to the istance of point_list without the last
 *                      element.
 */
void point_list::pop()
{
	point_node *tmp = head;
	while (tmp->pnext->pnext != NULL)
	{
		tmp = tmp->pnext;
	};
	delete tmp->pnext;
	tmp->pnext = NULL;
	tail = tmp;

	point_node *tl = head;
	int count = 0;
	while (tl != NULL)
	{
		count++;
		tl = tl->pnext;
	}
	size--;
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
	if (head == NULL)
    {
        head = e->head;
        tail = e->tail;
    }
    else
    {
        tail->pnext = e->head;
	    tail = e->tail;
    }
	size += e->size;
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
	head = NULL;
	tail = NULL;
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
	points = new point_list();
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

	Edge_boost self = Edge_boost(points->head->to_boost(),
								 points->tail->to_boost());
	Edge_boost e_boost = Edge_boost(e->points->head->to_boost(),
									e->points->tail->to_boost());
	vector<pt> inter;

	if (boost::geometry::intersects(self, e_boost))
	{
		boost::geometry::intersection(self, e_boost, inter);
		return new point_node(inter[0].x(), inter[0].y());
	}
	else
	{
		return NULL;
	}
};

/**
 * \fun middle_point()
 * This method computes the middle point of an edge instance.
 * @return point_node*: It is the pointer to the point_node instance
 *                      representing the middle_point.
 */
point_node *Edge::middle_point()
{
	double mid_x = ((points->head->x + points->tail->x)) / 2;
	double mid_y = ((points->head->y + points->tail->y)) / 2;
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
 * Checks whether the edge has one of the verteces of the provided polygons
 */
point_node *edge_has_vertex(Edge *e, polygon *pol)
{
    Edge_list *pol_edges = pol->edgify();
	Edge *e_iter = pol_edges->head;
	point_node *vertex = NULL;
	// bool has_edge = false;

	while (e_iter != NULL && vertex == NULL)
	{
		point_node *inter = e->intersection(e_iter);
		if (inter != NULL && pol->is_vertex(inter))
		{
			vertex = inter;
		}
		e_iter = e_iter->next;
	}
	return vertex;
}

Edge *Edge::copy()
{
	return new Edge(points->head, points->tail);
}

/**
 * \fun add_edge(Edge* e)
 * This method allows to add a new Edge instance to the list of edges.
 * @param e: Edge\*. It is the pointer to the Edge instance to add to the list.
 */
void Edge_list::add_edge(Edge *e)
{
	if (head == NULL)
	{
		head = e->copy();
		tail = head;
		size++;
		return;
	};

	tail->next = e->copy();
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
 * \fun ~Edge_list()
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
			if (show)
			{
				cout << "Plotting Line: "
					 << "[" << (n1->x * SCALE_1) + SCALE_2 << ":"
					 << (n1->y * -SCALE_1) + SCALE_2 << "]\t"
					 << "[" << (n2->x * SCALE_1) + SCALE_2 << ":"
					 << ((n2->y * -SCALE_1) + SCALE_2) << "]" << endl;
			};
			n1 = n1->pnext;
		}

		if (isPolygon)
		{ // close the polygon connecting the last point with the firts one
			line(arena, cv::Point((pl->tail->x * SCALE_1) + SCALE_2, (pl->tail->y * -SCALE_1) + SCALE_2),
				 cv::Point((pl->head->x * SCALE_1) + SCALE_2,
						   (pl->head->y * -SCALE_1) + SCALE_2),
				 colorline, thickness);
			if (show)
			{
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

/**
 * \fun
 * This function implements the rangeSymm operator.
 *
 * @param ang
 * @return double
 */
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
polygon::polygon(point_list *pls, string _id)
{
	if (pls->size >= 3)
	{
		pl = pls->copy();
		pt new_centroid;
		boost::geometry::centroid(pls->to_boost_polygon(), new_centroid);
		centroid = new point_node(new_centroid.x(), new_centroid.y());
		area = boost::geometry::area(pls->to_boost_polygon());
		id = _id;
	}
	else
	{
		throw std::invalid_argument("Error: A polygon can't have less"
									"than 3 points.\n");
		exit(-1);
	}
};

/**
 * \fun
 * This method recompute the centroid's coordinates
 * for the polygon
 */
void polygon::recompute_centroid()
{
	point_node *tmp = pl->head;
	double x_sum = 0;
	double y_sum = 0;
	while (tmp != NULL)
	{
		x_sum += tmp->x;
		y_sum += tmp->y;
		tmp = tmp->pnext;
	};
	centroid = new point_node(x_sum / pl->size, y_sum / pl->size);
};

/**
 * \fun
 * This method store in a map the common edge
 * with a cell
 *
 * @param s: string. Cell id
 * @param e: Edge *. Common edge
 */
void polygon::add_common_edge(string s, Edge *e)
{
	if (common_edges.count(s) == 0)
	{
		common_edges[s] = new Edge_list;
	}
	common_edges[s]->add_edge(e);
}

/**
 * \fun
 * This function merge two polygons.
 *
 * @param p1: polygon. First polygon.
 * @param p2: polygon. Second polygon.
 * @return polygon*: Merged polygon
 */
polygon *merge(polygon *p1, polygon *p2)
{
	if (p1 == NULL)
	{
		cout << "First polygon in merge is NULL" << endl;
	}
	if (p2 == NULL)
	{
		cout << "Second polygon in merge is NULL" << endl;
	}

	cout << "Merging: " << p1->id << " " << p2->id << endl;
	cout << "Edges sizes: " << p1->common_edges.size() << " "
		 << p2->common_edges.size() << endl;
	vector<Polygon_boost> merge_out;
	Polygon_boost P1 = p1->to_boost_polygon();
	Polygon_boost P2 = p2->to_boost_polygon();
	boost::geometry::union_(P1, P2, merge_out);
	polygon *result = boost_polygon_to_polygon(merge_out[merge_out.size() - 1]);

	map<string, Edge_list *>::const_iterator it = p1->common_edges.cbegin();
	while (it != p1->common_edges.cend())
	{
		Edge *tmp = it->second->head;
		while (tmp != NULL)
		{
			result->add_common_edge(it->first, tmp);
			tmp = tmp->next;
		}
		it++;
	}

	it = p2->common_edges.cbegin();
	while (it != p2->common_edges.cend())
	{
		Edge *tmp = it->second->head;
		while (tmp != NULL)
		{
			result->add_common_edge(it->first, tmp);
			tmp = tmp->next;
		}
		it++;
	}
	result->id = p1->id;
	return result;
}

point_list *polygon::intersections_with_edge(Edge *e)
{
	Edge_list *pol_edges = edgify();
	point_list *intersection_points = new point_list();

	Edge *it_e = pol_edges->head;
	while (it_e != NULL)
	{
		point_node *intersection = it_e->intersection(e);

		// Do not allow duplicates
		if (intersection != NULL && !intersection_points->is_in(intersection))
		{
			intersection_points->add_node(intersection->copy());
		}
		it_e = it_e->next;
	}
	return intersection_points;
}

bool polygon::is_vertex(point_node *p)
{
	point_node *temp = pl->head;
	while (temp != NULL)
	{
		if (*temp == *p)
		{
			cout << "Found Vertex" << endl;
			return true;
		}
		temp = temp->pnext;
	}
	return false;
}

/**
 * \fun
 * Destructor of the polygon
 */
polygon::~polygon()
{
	id = "NaN";
	pl->delete_list();
	pnext = NULL;
	area = 0.0;
	delete (centroid);
	common_edges.clear();
};

/**
 * \fun
 * This method takes the point_list of a polygon
 * and converts it in a list of Edges.
 *
 * @return Edge_list*: Converted edges list
 */
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

bool point_list::is_in(point_node *p)
{
	point_node *p_iter = head;
	while (p_iter != NULL)
	{
		if (*p == *p_iter)
		{
			return true;
		}
		else
		{
			p_iter = p_iter->pnext;
		}
	}
	return false;
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
	bool found_local_point = false;
	while (tmp_pol_1 != NULL)
	{
		found_local_point = false;
		point_node *tmp_pol_2 = p->pl->head;
		while (tmp_pol_2 != NULL && found_local_point == false)
		{
			double eps_mod_x = max(fabs(tmp_pol_1->x), fabs(tmp_pol_2->x));
			double eps_mod_y = max(fabs(tmp_pol_1->y), fabs(tmp_pol_2->y));

			/*
			cout << "X: " << tmp_pol_1->x << " - " << tmp_pol_2->x
				 << " = " << tmp_pol_1->x - tmp_pol_2->x << " <= "
				 << epsilon*eps_mod_x << endl;
			cout << "Y: " << tmp_pol_1->y << " - " << tmp_pol_2->y
				 << " = " << tmp_pol_1->y - tmp_pol_2->y << " <= "
				 << epsilon*eps_mod_y << " ";
			*/

			if (fabs(tmp_pol_1->x - tmp_pol_2->x) <= epsilon * eps_mod_x &&
				fabs(tmp_pol_1->y - tmp_pol_2->y) <= epsilon * eps_mod_y)
			{
				points_in_common += 1;
				found_local_point = true;
				if (common_edges.count(p->id) == 0)
				{
					common_edges[p->id] = new Edge_list;
				}
				point_node *p1 = new point_node(tmp_pol_1->x, tmp_pol_1->y);
				point_node *p2 = new point_node(tmp_pol_2->x, tmp_pol_2->y);
				common_edges[p->id]->add_edge(new Edge(p1, p2));
			};
			tmp_pol_2 = tmp_pol_2->pnext;
		};
		tmp_pol_1 = tmp_pol_1->pnext;
	};
	return points_in_common;
}

/**
 * \fun
 * This method print the information about the
 * polygon.
 *
 */
void polygon::info()
{
	cout << "polygon: " << id << endl;
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
		head = p->copy();
		tail = head;
	}
	else
	{
		tail->pnext = p->copy();
		tail = tail->pnext;
	}
	size += 1;
};

/**
 * \fun
 * This method append to a list of polygons another list of polygons,
 * creating a merged ones.
 *
 * @param p: list_of_polygons. Merged list of polygons.
 */
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
	return atan2(y - yc, x - xc);
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
	if (th < th0 || th > thf)
	{
		return false;
	}
	return true;
}

/**
 * \fun
 * This method return a copy of the point list.
 *
 * @return point_list*: Copy of the point list
 */
point_list *point_list::copy()
{
	point_list *copy = new point_list();
	point_node *node_iter = head;
	while (node_iter != NULL)
	{
		copy->add_node(node_iter->copy());
		node_iter = node_iter->pnext;
	}
	/*
	copy->head = head->copy();
	copy->tail = tail->copy();
	copy->x_min = x_min;
	copy->x_max = x_max;
	copy->y_min = y_min;
	copy->y_max = y_max;
	copy->size = size;
	*/
	return copy;
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
polygon *boost_polygon_to_polygon(Polygon_boost p, string _id)
{
	point_list *new_pol_list = boost_polygon_to_point_list(p);
	pt new_centroid;
	boost::geometry::centroid(p, new_centroid);

	polygon *new_pol = new polygon(new_pol_list);
	new_pol->centroid = new point_node(new_centroid.x(), new_centroid.y());
	new_pol->area = boost::geometry::area(p);
	new_pol->id = _id;
	return new_pol;
};

/**
 * \fun
 * This method return a copy of the polygon.
 *
 * @return polygon*: Polygon's copy
 */
polygon *polygon::copy()
{
	polygon *copy = new polygon(pl->copy(), id);
	copy->centroid = centroid->copy();
	copy->pnext = NULL;
	copy->area = area;
	copy->common_edges = common_edges;
	return copy;
}

double cross2D(double *v1, double *v2)
{
	return v1[0] * v2[1] - v1[1] * v2[0];
}

double dot2D(double *v1, double *v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1];
}

/**
 * \fun
 * This function tells if a point is part of the
 * line formed by other 2 points.
 *
 * @param p1: double. First point.
 * @param p2: double. Point to check.
 * @param p3: double. Second point.
 * @returns: True if on segment. False otherwise
 */
bool onSegment(point_node *p1, point_node *p2, point_node *p3)
{
	if (p2->x <= max(p1->x, p3->x) && p2->x >= min(p1->x, p3->x) &&
		p2->y <= max(p1->y, p3->y) && p2->y >= min(p1->y, p3->y))
		return true;

	return false;
}

// onSegment potrebbe non servire piu.
/**
 * \fun
 * This function tells if three points are clockwise
 * or counterclockwise.
 *
 * @param p1: point_node *. First point.
 * @param p2: point_node *. Second point.
 * @param p3: point_node *. Third point.
 * @returns int: 0 if collinear, 1 if clockwise, 2 counterclockwise.
 */
int orientation(point_node *p1, point_node *p2, point_node *p3)
{
	// See https://www.geeksforgeeks.org/orientation-3-ordered-points/
	// for details of below formula.
	int val = (p2->y - p1->y) * (p3->x - p2->x) -
			  (p2->x - p1->x) * (p3->y - p2->y);

	if (val == 0)
		return 0; // collinear

	return (val > 0) ? 1 : 2; // clock or counterclock wise
}

/**
 * \fun
 * This function calculate the intersection points between two lines.
 *
 * @param p1: First point of the first line.
 * @param p2: Second point of the first line.
 * @param p3: First point of the second line.
 * @param p4: Second point of the second line.
 * @returns: true if intersects, false otherwise
 */
bool intersLineLine(point_node *p1, point_node *p2, point_node *p3, point_node *p4)
{
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, p2, p2);
	int o2 = orientation(p1, p2, p4);
	int o3 = orientation(p3, p4, p1);
	int o4 = orientation(p3, p4, p2);

	// General case
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases
	// p1, q1 and p2 are collinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p3, p2))
		return true;

	// p1, q1 and q2 are collinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, p4, p2))
		return true;

	// p2, q2 and p1 are collinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p3, p1, p4))
		return true;

	// p2, q2 and q1 are collinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p3, p2, p4))
		return true;

	return false; // Doesn't fall in any of the above cases
}

/**
 * \fun
 * This function return the intersection points between a circle and a line.
 *
 * @param a: double. x-coordinate of the center of the circle
 * @param b: double. y-coordinate of the center of the circle
 * @param r: double. radius of the circle
 * @param x1: double. x-coordinate of the first point of the line
 * @param y1: double. y-coordinate of the first point of the line
 * @param x2: double. x-coordinate of the second point of the line
 * @param y2: double. y-coordinate of the second point of the line
 * @return tuple<point_list *, double_list *>
 */
tuple<point_list *, double_list *> intersCircleLine(double a, double b, double r, double x1, double y1, double x2, double y2)
{
	double p1, p2, p3, p4, p5, p6;
	p1 = 2 * x1 * x2;
	p2 = 2 * y1 * y2;
	p3 = 2 * a * x1;
	p4 = 2 * a * x2;
	p5 = 2 * b * y1;
	p6 = 2 * b * y2;

	double c1, c2, c3;
	c1 = pow(x1, 2) + pow(x2, 2) - p1 + pow(y1, 2) + pow(y2, 2) - p2;
	c2 = -2 * pow(x2, 2) + p1 - p3 + p4 - 2 * pow(y2, 2) + p2 - p5 + p6;
	c3 = pow(x2, 2) - p4 + pow(a, 2) + pow(y2, 2) - p6 + pow(b, 2) - pow(r, 2);

	double delta = pow(c2, 2) - 4 * c1 * c3;
	point_list *pts = new point_list;
	double_list *t = new double_list;

	double t1, t2, x, y;
	double deltaSq;
	if (delta < 0)
	{
		double_list *dnull = NULL;
		point_list *pnull = NULL;
		// cout << "DELTA: " << delta << endl;
		return make_tuple(pnull, dnull);
	}
	else
	{
		if (delta > 0)
		{
			deltaSq = sqrt(delta);
			t1 = (-c2 + deltaSq) / (2 * c1);
			t2 = (-c2 - deltaSq) / (2 * c1);
		}
		else
		{
			t1 = -c2 / (2 * c1);
			t2 = t1;
		}
	}
	if (t1 >= 0 && t1 <= 1)
	{
		x = x1 * t1 + x2 * (1 - t1);
		y = y1 * t1 + y2 * (1 - t1);
		pts->add_node(new point_node(x, y));
		t->add_node(new double_node(t1));
		// cout << "NODO AGGIUNTO\n";
	}
	if (t2 >= 0 && t2 <= 1 && t2 != t1)
	{
		x = x1 * t2 + x2 * (1 - t2);
		y = y1 * t2 + y2 * (1 - t2);
		cv::Point pt(x, y);
		// cout << typeid(pt.x).name() << endl;
		pts->add_node(new point_node(x, y));

		t->add_node(new double_node(t2));
		// cout << "NODO AGGIUNTO\n";
	}
	sort(t, pts);
	return make_tuple(pts, t);
}

/**
 * \fun
 * This function check if 2 centroids are in line of sight.
 * If not the middel point of the common edge between the 2 cells
 * is returned to help to modify the dubins path. To check that is necessary
 * iterate all the edges of all the obstacles with the segment formed by the 2
 * centroids and see if they intersecates.
 *
 * @param p1: point_node *. First centroid.
 * @param p2: point_node *. Second centroid.
 * @param pol: polygon *. First polygon of the obstacles list.
 * @return point_node*. Null if the centroids are in los,
 * 						middle point of the common edge otherwise.
 */
bool los(point_node *p1, point_node *p2, polygon *pol, vector<Edge *> *sweep_line)
{
	polygon *tmp = pol->copy();
	bool los = true;
	Edge *los_e = new Edge(p1,p2);
	while (tmp != NULL && los == true)
	{
		point_node *pt1 = tmp->pl->head;
		while (pt1 != NULL && los == true)
		{
			point_node *pt2;
			if (pt1->pnext == NULL)
			{
				pt2 = tmp->pl->head;
			}
			else
			{
				pt2 = pt1->pnext;
			}
			Edge * pol_e = new Edge(pt1,pt2);
			if (los_e->intersection(pol_e) != NULL)
			{
				los = false;
			}

			pt1 = pt1->pnext;
		}
		tmp = tmp->pnext;
	}

	if (sweep_line != NULL)
	{
		vector<Edge *> s_e = *sweep_line;
		int i=0;
		while(i < s_e.size() && los == true)
		{
			if (los_e->intersection(s_e[i]) != NULL	)
			{
				los = false;
			}
			i++;
		}
	}
	return los;
}

/**
 * \fun
 * This function find the common edge between two polygons
 *
 * @param p1: polygon *. First polygon.
 * @param p2: polygon *. Second polygon.
 * @return Edge*: Common edge. Null if doesn't exits.
 */
Edge *find_common_edge(polygon *p1, polygon *p2)
{
	Edge_list *e1 = p1->edgify();
	Edge_list *e2 = p2->edgify();

	Edge *tmp = e1->head;

	while (tmp != NULL)
	{
		Edge *tmp_2 = e2->head;
		while (tmp_2 != NULL)
		{
			point_node *p11 = tmp->points->head;
			point_node *p12 = tmp->points->tail;
			point_node *p21 = tmp_2->points->head;
			point_node *p22 = tmp_2->points->tail;

			if ((*p11 == *p21 && *p12 == *p22) || (*p11 == *p22 && *p12 == *p21))
			{
				cout << "Found common edge between " << p1->id << " and "
					 << p2->id << endl;
				return new Edge(new point_node(p11->x, p11->y),
								new point_node(p12->x, p12->y));
			}
			tmp_2 = tmp_2->next;
		}
		tmp = tmp->next;
	}
	return NULL;
}

list_of_polygons *subset_over_middle_point(polygon *p)
{
	list_of_polygons *tmp_output = new list_of_polygons();

	Edge_list *e_list = p->edgify();

	cout << "Polygon " << p->id << " has " << p->pl->size << " points" << endl;
	cout << "Polygon " << p->id << " has " << e_list->size << " edges" << endl;

	Edge *tmp = e_list->head;

	Edge *start = e_list->head;
	int num = 0;
	while (start != NULL)
	{
		Edge *end = NULL;
		if (start->next == NULL)
		{
			end = e_list->head;
		}
		else
		{
			end = start->next;
		};

		point_list *pl_temp = new point_list();
		pl_temp->add_node(new point_node(p->centroid->x, p->centroid->y));
		pl_temp->add_node(start->middle_point());
		pl_temp->add_node(start->points->tail);
		pl_temp->add_node(end->middle_point());

		polygon *cell = new polygon(pl_temp, p->id + "_" + to_string(num));

		tmp_output->add_polygon(cell);

		start = start->next;
		num++;
	};
	return tmp_output;
}

/**
 * Subtract a vector of polygons from another one.
 * @param arena: vector<Polygon_boost>. Is the vector of polygons from which
 * the other polygons will be subtracted.
 * @param obstacles: vector<Polygon_boost>. Is the vector of polygons that will
 * be subtracted.
 */
vector<Polygon_boost> difference_of_vectors(vector<Polygon_boost> arena,
											vector<Polygon_boost> obstacles)
{

	if (arena.size() == 0)
	{
		cout << "Arena vector is empty." << endl;
	};
	if (obstacles.size() == 0)
	{
		cout << "Obstacle vector is empty." << endl;
	};

	vector<Polygon_boost> output;
	vector<Polygon_boost> tmp_output;

	int prev_output = 0;
	int arena_size = arena.size();
	int arena_ob_size = obstacles.size();

	for (int i = 0; i < arena_size; i++) // i is the index of the cell
	{
		int j = 0; // Index of the obstacle
		bool interrupt_ob = false;
		while (j < arena_ob_size && interrupt_ob == false)
		{
			// Apply only if cells are outside the obstacles
			if (boost::geometry::covered_by(arena[i], obstacles[j]) == false)
			{
				if (boost::geometry::intersects(arena[i], obstacles[j]))
				{
					// cout << "Intersects" << endl;
					boost::geometry::difference(arena[i], obstacles[j], output);
					int diff = output.size() - prev_output;
					prev_output = output.size();

					// cout << "Output size: " << output.size() << endl;
					// cout << "Difference: " << diff << " Size output: "
					// 	 << prev_output << endl;

					arena[i] = output[output.size() - 1];

					if (diff > 1)
					{
						// cout << "In diff > 1" << endl;
						// tmp_output = arena;
						for (int k = 1; k < diff; k++)
						{
							int output_idx = output.size() - 1 - k;
							vector<Polygon_boost>::iterator it;
							// it = tmp_output.begin();
							it = arena.begin();
							// tmp_output
							// arena.insert(it + i + k, output[output_idx]);
							arena.push_back(output[output_idx]);
							arena_size++;
						};
						// arena = tmp_output;
					};
				};
			}
			else
			{
				// Delete cells covered by obstacles;
				// cout << "Covered" << endl;
				arena.erase(arena.begin() + i);
				i--;
				arena_size--;
				interrupt_ob = true;
			};
			j++;
		};
	};
	return arena;
};

string PDDL_conditional_cost(string el1, string el2, double cost,
                             string cost_name)
{
	string cost_PDDL = "\n\t\t\t\t\t( when"
        "\n\t\t\t\t\t\t( or"
        "\n\t\t\t\t\t\t\t( and"
        "\n\t\t\t\t\t\t\t\t( is_" +
        el1 + " ?loc_start )"
        "\n\t\t\t\t\t\t\t\t( is_" +
        el2 + " ?loc_end)"
        "\n\t\t\t\t\t\t\t)"
        "\n\t\t\t\t\t\t\t( and"
        "\n\t\t\t\t\t\t\t\t( is_" +
        el2 + " ?loc_start )"
        "\n\t\t\t\t\t\t\t\t( is_" +
        el1 + " ?loc_end)"
        "\n\t\t\t\t\t\t\t)"
        "\n\t\t\t\t\t\t)"
        "\n\t\t\t\t\t\t( increase (" + cost_name + ") " +
        to_string(cost) + " )"
        "\n\t\t\t\t\t)";
	return cost_PDDL;
}

bool compare_doubles(double a, double b)
{
	double diff = fabs(fabs(b) - fabs(a));
	double threshold = 1e-6 * max(fabs(a), fabs(b));
	if (diff < threshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool equal_point_nodes(point_node *a, point_node *b)
{
	if (a == NULL || b == NULL)
	{
		cout << "One of input points is NULL" << endl;
		return false;
	}

	if (compare_doubles(a->x, b->x) && compare_doubles(a->y, b->y))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool point_belong_to_edge(point_node *p, Edge *e)
{
	point_node *start_e = e->points->head;
	point_node *end_e = e->points->tail;

	double *x = NULL;
	double *y = NULL;

	double diff_y = start_e->y - end_e->y;
	double diff_x = start_e->x - end_e->x;

	if (diff_y == 0)
	{
		y = new double(start_e->y);
	}

	if (diff_x == 0)
	{
		x = new double(start_e->x);
	}

	if (y == NULL)
	{
		y = new double((p->y - end_e->y) / diff_y);
	}

	if (x == NULL)
	{
		x = new double((p->x - end_e->x) / diff_x);
	}

	cout << *y << " " << *x << endl;

	if (compare_doubles(*y, *x))
	{
		return true;
	}

	return false;
}

point_list * point_list::orderify(int axis)
{
    if (size == 2)
    {
        return order_pair_ascending(head, tail, axis);
    }
    else
    {
        cout << "Yet to be implemented -> return NULL" << endl;
        return NULL;
    }
}

/*
 * int axis:
 * - 0 = X
 * - 1 = Y
 */
point_list * order_pair_ascending(point_node * a, point_node * b, int axis)
{
    point_list * ordered_pair = new point_list();
    double quantity_a, quantity_b;
    point_node * lower = NULL;
    point_node * higher = NULL;

    switch(axis)
    {
        case 0:
            {
                quantity_a = a->x;
                quantity_b = b->x;
                break;
            }
        case 1:
            {
                quantity_a = a->y;
                quantity_b = b->y;
                break;
            }
    }
    if (quantity_a < quantity_b)
    {
        lower = a;
        higher = b;
    }
    else
    {
        lower = b;
        higher = a;
    }
    ordered_pair->add_node(lower);
    ordered_pair->add_node(higher);
    
    return ordered_pair;
}

tuple<map<string, pair<double, double>>, int> string_vectors_intersection(vector<string> a,
                                                                         vector<string> b,
                                                                         bool reverse)
{
    map<string, pair<double, double>> int_indexes;
    set<string> unique_elements;

    if (a.size() == 0 || b.size() == 0)
    {
        cout << "One vector is empty -> no intersections" << endl;
    }

    for(int i=0; i<a.size(); i++)
    {
        int index = i;
        if(reverse = true)
        {
            int index = b.size()-i;
        }

        for(int j=0; j<b.size(); j++)
        {
            if (a[index] == b[j])
            {
                unique_elements.insert(b[j]);
                int_indexes[b[j]] = pair<double, double>(i, j);
            }
        }
    }
    return std::make_tuple(int_indexes, unique_elements.size());
}

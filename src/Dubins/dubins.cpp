#include "dubins.h"

/*--------------------START DUBINS.H----------------------------------*/
tuple<int, curve> dubins(double x0, double y0, double th0, double xf, double yf, double thf)
{
	double sc_th0, sc_thf, sc_Kmax, lambda;
	tie(sc_th0, sc_thf, sc_Kmax, lambda) = scaleToStandard(x0, y0, th0, xf, yf, thf);
	int ksigns[6][3] = {{1, 0, 1}, {-1, 0, -1}, {1, 0, -1}, {-1, 0, 1}, {-1, 1, -1}, {1, -1, 1}};
	int pidx = -1;
	double L = INFINITY;
	curve c;

	double sc_s1, sc_s2, sc_s3;
	double s1, s2, s3;
	bool ok;
	for (int i = 0; i < 6; i++)
	{
		double sc_s1_c, sc_s2_c, sc_s3_c;
		switch (i)
		{
		case 0:
			tie(ok, sc_s1_c, sc_s2_c, sc_s3_c) = LSL(sc_th0, sc_thf, sc_Kmax);
			break;
		case 1:
			tie(ok, sc_s1_c, sc_s2_c, sc_s3_c) = RSR(sc_th0, sc_thf, sc_Kmax);
			break;
		case 2:
			tie(ok, sc_s1_c, sc_s2_c, sc_s3_c) = LSR(sc_th0, sc_thf, sc_Kmax);
			break;
		case 3:
			tie(ok, sc_s1_c, sc_s2_c, sc_s3_c) = RSL(sc_th0, sc_thf, sc_Kmax);
			break;
		case 4:
			tie(ok, sc_s1_c, sc_s2_c, sc_s3_c) = RLR(sc_th0, sc_thf, sc_Kmax);
			break;
		case 5:
			tie(ok, sc_s1_c, sc_s2_c, sc_s3_c) = LRL(sc_th0, sc_thf, sc_Kmax);
			break;
		}
		double Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
		if (ok && Lcur < L)
		{
			L = Lcur;
			sc_s1 = sc_s1_c;
			sc_s2 = sc_s2_c;
			sc_s3 = sc_s3_c;
			pidx = i;
		}
	}

	ok = false;
	if (pidx > 0)
	{
		ok = true;
		tie(s1, s2, s3) = scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3);
		c = dubinscurve(x0, y0, th0, s1, s2, s3,
						ksigns[pidx][0] * KMAX, ksigns[pidx][1] * KMAX,
						ksigns[pidx][2] * KMAX);
		assert(check(sc_s1, ksigns[pidx][0] * sc_Kmax, sc_s2,
					 ksigns[pidx][1] * sc_Kmax, sc_s3,
					 ksigns[pidx][2] * sc_Kmax, sc_th0, sc_thf));
	}
	return make_tuple(pidx, c);
}

/**
 * \fun
 * This method creates a dubins arc.
 *
 * @param x0: double. x-coordinate of the starting point of the arc
 * @param y0: double. y-coordinate of the starting point of the arc
 * @param th0: double. Starting angle
 * @param k: double. Curvature angle
 * @param L: Length of the arc
 * @return arc
 */
arc dubinsarc(double x0, double y0, double th0, double k, double L)
{
	arc c;
	c.x0 = x0;
	c.y0 = y0;
	c.th0 = th0;
	c.k = k;
	c.L = L;
	tie(c.xf, c.yf, c.thf, c.xc, c.yc, c.r) = circline(L, x0, y0, k, th0, true);
	return c;
}

/**
 * \fun
 * This function creates a dubins curve.
 *
 * @param x0: double. x-coordinate of the starting point of the curve
 * @param y0: double. y-coordinate of the starting point of the curve
 * @param th0: double. Starting angle.
 * @param s1: double. Lenght of the first arc.
 * @param s2: double. Lenght of the second arc.
 * @param s3: double. Lenght of the third arc.
 * @param k0: double. curvature angle of the first arc.
 * @param k1: double. curvature angle of the second arc.
 * @param k2: double. curvature angle of the third arc.
 * @return curve
 */
curve dubinscurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2)
{
	curve d;
	d.a1 = dubinsarc(x0, y0, th0, k0, s1);
	d.a2 = dubinsarc(d.a1.xf, d.a1.yf, d.a1.thf, k1, s2);
	d.a3 = dubinsarc(d.a2.xf, d.a2.yf, d.a2.thf, k2, s3);
	d.L = d.a1.L + d.a2.L + d.a3.L;
	return d;
}

tuple<double, double, double, double> scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf)
{
	double dx = xf - x0;
	double dy = yf - y0;
	double phi = atan2(dy, dx);
	double lambda = hypot(dx, dy) / 2;
	double sc_Kmax = KMAX * lambda;
	double sc_th0 = mod2pi(th0 - phi);
	double sc_thf = mod2pi(thf - phi);
	return make_tuple(sc_th0, sc_thf, sc_Kmax, lambda);
}

tuple<double, double, double> scaleFromStandard(double lambda, double sc_s1, double sc_s2, double sc_s3)
{ // invert scaling transform (back to original data)
	double s1 = sc_s1 * lambda;
	double s2 = sc_s2 * lambda;
	double s3 = sc_s3 * lambda;
	return make_tuple(s1, s2, s3);
}

tuple<bool, double, double, double> LSL(double sc_th0, double sc_thf, double sc_Kmax)
{
	double invK, C, S, temp1, temp2, sc_s1, sc_s2, sc_s3;
	bool ok;
	invK = 1 / sc_Kmax;
	C = cos(sc_thf) - cos(sc_th0);
	S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
	temp1 = atan2(C, S);
	sc_s1 = invK * mod2pi(temp1 - sc_th0);
	temp2 = 2 + 4 * pow(sc_Kmax, 2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
	if (temp2 < 0)
		return make_tuple(false, 0, 0, 0);

	sc_s2 = invK * sqrt(temp2);
	sc_s3 = invK * mod2pi(sc_thf - temp1);
	ok = true;
	return make_tuple(ok, sc_s1, sc_s2, sc_s3);
}
tuple<bool, double, double, double> RSR(double sc_th0, double sc_thf, double sc_Kmax)
{
	double invK, C, S, temp1, temp2, sc_s1, sc_s2, sc_s3;
	bool ok;
	invK = 1 / sc_Kmax;
	C = cos(sc_th0) - cos(sc_thf);
	S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
	temp1 = atan2(C, S);
	sc_s1 = invK * mod2pi(sc_th0 - temp1);
	temp2 = 2 + 4 * pow(sc_Kmax, 2) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
	if (temp2 < 0)
		return make_tuple(false, 0, 0, 0);
	sc_s2 = invK * sqrt(temp2);
	sc_s3 = invK * mod2pi(temp1 - sc_thf);
	ok = true;
	return make_tuple(ok, sc_s1, sc_s2, sc_s3);
}
tuple<bool, double, double, double> LSR(double sc_th0, double sc_thf, double sc_Kmax)
{
	double invK, C, S, temp1, temp2, temp3, sc_s1, sc_s2, sc_s3;
	bool ok;
	invK = 1 / sc_Kmax;
	C = cos(sc_th0) + cos(sc_thf);
	S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
	temp1 = atan2(-C, S);
	temp3 = 4 * pow(sc_Kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
	if (temp3 < 0)
		return make_tuple(false, 0, 0, 0);
	sc_s2 = invK * sqrt(temp3);
	temp2 = -atan2(-2, sc_s2 * sc_Kmax);
	sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
	sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
	ok = true;
	return make_tuple(ok, sc_s1, sc_s2, sc_s3);
}

tuple<bool, double, double, double> RSL(double sc_th0, double sc_thf, double sc_Kmax)
{
	double invK, C, S, temp1, temp2, temp3, sc_s1, sc_s2, sc_s3;
	bool ok;
	invK = 1 / sc_Kmax;
	C = cos(sc_th0) + cos(sc_thf);
	S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
	temp1 = atan2(C, S);
	temp3 = 4 * pow(sc_Kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
	if (temp3 < 0)
		return make_tuple(false, 0, 0, 0);
	sc_s2 = invK * sqrt(temp3);
	temp2 = atan2(2, sc_s2 * sc_Kmax);
	sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
	sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);
	ok = true;
	return make_tuple(ok, sc_s1, sc_s2, sc_s3);
}

tuple<bool, double, double, double> RLR(double sc_th0, double sc_thf, double sc_Kmax)
{
	double invK, C, S, temp1, temp2, sc_s1, sc_s2, sc_s3;
	bool ok;
	invK = 1 / sc_Kmax;
	C = cos(sc_th0) - cos(sc_thf);
	S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
	temp1 = atan2(C, S);
	temp2 = 0.125 * (6 - 4 * pow(sc_Kmax, 2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
	if (abs(temp2) > 1)
		return make_tuple(false, 0, 0, 0);
	sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
	sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_Kmax);
	sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2 - sc_s1));
	ok = true;
	return make_tuple(ok, sc_s1, sc_s2, sc_s3);
}

tuple<bool, double, double, double> LRL(double sc_th0, double sc_thf, double sc_Kmax)
{
	double invK, C, S, temp1, temp2, sc_s1, sc_s2, sc_s3;
	bool ok;
	invK = 1 / sc_Kmax;
	C = cos(sc_thf) - cos(sc_th0);
	S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
	temp1 = atan2(C, S);
	temp2 = 0.125 * (6 - 4 * pow(sc_Kmax, 2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
	if (abs(temp2) > 1)
		return make_tuple(false, 0, 0, 0);
	sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
	sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_Kmax);
	sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2 - sc_s1));
	ok = true;
	return make_tuple(ok, sc_s1, sc_s2, sc_s3);
}

/**
 * \fun
 * This function calculate the coordinates of a point in a circle and it's center's coordinates.
 *
 * @param s: double. Distance between the point to calculate and the starting point of a circle.
 * @param x0: double. x-coordinate of the starting point.
 * @param y0: double. y-coordinate of the starting point of the angle.
 * @param k: double. Angle's curvature.
 * @param th0: double. Starting angle.
 * @param get_center: bool. Bool to know if is necessary to calculate the center coordinates of the circle
 * @returns tuple<double x, double y, double th, double xc, double yc, double r>: Coordinates of the point and the center of the circle and it's radius.
 */
tuple<double, double, double, double, double, double> circline(double s, double x0, double y0, double k, double th0, bool get_center)
{
	double x, y, th, xc, yc, r;
	x = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2);
	y = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s / 2);
	th = mod2pi(th0 + k * s);

	// Finding a third point included in the arc.
	// It's necessary to find the center of the circle
	double x1, y1;
	double s1 = s / 2;
	x1 = x0 + s1 * sinc(k * s1 / 2.0) * cos(th0 + k * s1 / 2);
	y1 = y0 + s1 * sinc(k * s1 / 2.0) * sin(th0 + k * s1 / 2);

	// calculation of the circle center
	if (get_center)
		tie(xc, yc, r) = get_circle_center(x0, y0, x1, y1, x, y);
	else
		xc = 0, yc = 0, r = 0;

	return make_tuple(x, y, th, xc, yc, r);
}

/**
 * \fun
 * This function plot a dubins curve in a given image
 *
 * @param d: curve. Dubins curve to plot.
 * @param c1: arc. Color of the first arc.
 * @param c2: arc. Color of the second arc.
 * @param c3: arc. Color of the third arc.
 * @param arena: Mat. Img where to plot the curve.
 * @return Mat: Plotted img
 */
Mat plotdubins(curve d, string c1, string c2, string c3, Mat arena)
{
	arena = plotarc(d.a1, c1, arena);
	arena = plotarc(d.a2, c2, arena);
	arena = plotarc(d.a3, c3, arena);

	return arena;
}

/**
 * \fun
 * This function plot a given arc in the given img. To plot the arc, it is sectioned into 100 smaller arcs.
 *
 * @param a: arc. Arc to plot.
 * @param c: string. Color to use for the plot.
 * @param img: Mat. Img where to plot.
 * @return Mat: plotted img.
 */
Mat plotarc(arc a, string c, Mat img)
{
	int npts = 100;
	double th;
	cv::Point pts[npts];
	point_list *pl = new point_list;
	for (int i = 0; i < npts; i++)
	{
		double x, y, s, xc, yc, r;
		s = a.L / npts * i;
		tie(x, y, th, xc, yc, r) = circline(s, a.x0, a.y0, a.k, a.th0, false);
		pl->add_node(new point_node(x, y));
	}

	int r = 0, g = 0, b = 0;
	if (!strcmp(c.c_str(), "r") || !strcmp(c.c_str(), "red")) // strcmp() returns 0 if the strings are equal
	{
		r = 0;
		g = 0;
		b = 255;
	}
	else if (!strcmp(c.c_str(), "g") || !strcmp(c.c_str(), "green"))
	{
		r = 0;
		g = 255;
		b = 0;
	}
	else if (!strcmp(c.c_str(), "b") || !strcmp(c.c_str(), "blue"))
	{
		r = 0;
		g = 0;
		b = 255;
	}
	else if (!strcmp(c.c_str(), "y") || !strcmp(c.c_str(), "yellow"))
	{
		r = 255;
		g = 234;
		b = 128;
	}

	img = plot_points(pl, img, Scalar(b, g, r), false);
	pl->delete_list();
	return img;
}

/**
 * \fun
 * This function calculate the optimized arriving angle for the dubins curves for all the path to follow.
 * The opti theta is calculated as the angle formed by the line that joins two points.
 *
 * @param xpath: vector<double>. x-coordinates of the centroids of the free space cells of the path to follow
 * @param ypath: vector<double>. y-coordinates of the centroids of the free space cells of the path to follow
 * @return vector<double>
 */
vector<double> opti_theta(vector<double> xpath, vector<double> ypath)
{
	vector<double> thpath;
	if (xpath.size() > 0)
	{
		int size = xpath.size();
		for (int i = 0; i < size - 1; i++)
		{
			thpath.push_back(get_angle(xpath[i], ypath[i],
									   xpath[i + 1], ypath[i + 1]));
		}
		thpath.push_back(thpath[thpath.size() - 1]);
	}
	return thpath;
}

/**
 * \fun
 * This function return the center of a circle given three points of it. The source of the code is taken from:
 * https://www.geeksforgeeks.org/equation-of-circle-when-three-points-on-the-circle-are-given/
 *
 * @param x1: double. x-coordinate of the first point of the circonference
 * @param y1: double. y-coordinate of the first point of the circonference
 * @param x2: double. x-coordinate of the second point of the circonference
 * @param y2: double. y-coordinate of the second point of the circonference
 * @param x3: double. x-coordinate of the third point of the circonference
 * @param y3: double. y-coordinate of the third point of the circonference
 * @return tuple<double, double, double>
 */
tuple<double, double, double> get_circle_center(double x1, double y1, double x2, double y2, double x3, double y3)
{
	double x12 = x1 - x2;
	double x13 = x1 - x3;

	double y12 = y1 - y2;
	double y13 = y1 - y3;

	double y31 = y3 - y1;
	double y21 = y2 - y1;

	double x31 = x3 - x1;
	double x21 = x2 - x1;

	// x1^2 - x3^2
	double sx13 = pow(x1, 2) - pow(x3, 2);

	// y1^2 - y3^2
	double sy13 = pow(y1, 2) - pow(y3, 2);

	double sx21 = pow(x2, 2) - pow(x1, 2);
	double sy21 = pow(y2, 2) - pow(y1, 2);

	double f = ((sx13) * (x12) + (sy13) * (x12) + (sx21) * (x13) + (sy21) * (x13)) / (2 * ((y31) * (x12) - (y21) * (x13)));
	double g = ((sx13) * (y12) + (sy13) * (y12) + (sx21) * (y13) + (sy21) * (y13)) / (2 * ((x31) * (y12) - (x21) * (y13)));

	double c = -pow(x1, 2) - pow(y1, 2) - 2 * g * x1 - 2 * f * y1;

	// eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
	// where centre is (h = -g, k = -f) and radius r
	// as r^2 = h^2 + k^2 - c
	double h = -g;
	double k = -f;

	double sqr_of_r = h * h + k * k - c;

	// r is the radius
	double r = sqrt(sqr_of_r);

	return make_tuple(h, k, r);
}

/**
 * \fun
 * This function discretize a range of angles around a given angle. The angles are taken each 1°.
 *
 * @param starting_angle: double. Angle where to discretize around.
 * @param search_angle: Range where to discretize.
 * @return vector<double>
 */
vector<double> theta_discretization(double starting_angle, double search_angle)
{

	vector<double> plausible_theta;
	plausible_theta.push_back(starting_angle);
	double res_steps = 10 * (M_PI / 180); // 1 degree converted in radiants

	double theta = starting_angle - (search_angle / 2);

	int i = -1;
	while (theta <= starting_angle)
	{
		double val = starting_angle + (i * theta);
		plausible_theta.push_back(val);

		if (i > 0)
			theta += res_steps;

		i = -i;
	}
	return plausible_theta;
}

/**
 * \fun
 * This function return a dubins curve that doesn't intersect with any corner and it's pidx.
 * This function calculates the dubins curve for each angle discretized in a given range. Each dubins curve is checked that it does not intersect
 * with any corner of the arena and any corner of any obstacle. If an intersection the curve is discarded otherwise its length is checked to see
 * if it is the shortest of all the curves found so far. If yes it is saved with its pidx. Before store the curve, it's arriving angle is checked
 * that it is not present in a list of angles that cannot be used. a If any curve is founded the searching angle become the opposite of the area used.
 * If again no curve is founded then the pidx <= 0 is returned as information about it.
 *
 * @param x0: double. x-coordinate of the starting point.
 * @param y0: double. y-coordinate of the starting point.
 * @param th0: double. Initial angle.
 * @param xf: double. x-coordinate of the final point.
 * @param yf: double. y-coordinate of the final point.
 * @param thf: double. Arriving angle.
 * @param arena_limits: List of the points of the arena's limits.
 * @param obstacle: polygon. First element of the polygon list in the arena.
 * @param search_angle: double. Area where discretize the angles.
 * @param used_th: List of the already used final angle that must not to be re-used
 * @return tuple<curve, int>
 */
tuple<curve, int> dubins_no_inter(double x0, double y0, double th0,
								  double xf, double yf, double thf,
								  point_list *arena_limits, polygon *obstacle)
{
	curve c;
	int pidx;

	bool intersection = false;

	// vector<double> angles = theta_discretization(initial_th, search_angle);

	tie(pidx, c) = dubins(x0, y0, th0, xf, yf, thf);

	if (pidx > 0)
	{
		// Boolean that check the intersection for all the three arcs of a curve
		bool inter_a1, inter_a2, inter_a3;

		/* Search an intersection between the arcs of the dubins curve and the arena */
		point_node *pt_arena_1 = arena_limits->head;

		while (pt_arena_1 != NULL && intersection == false)
		{
			point_node *pt_arena_2;
			if (pt_arena_1->pnext == NULL)
			{
				pt_arena_2 = arena_limits->head;
			}
			else
			{
				pt_arena_2 = pt_arena_1->pnext;
			}

			inter_a1 = find_intersection(c.a1, pt_arena_1, pt_arena_2);
			inter_a2 = find_intersection(c.a2, pt_arena_1, pt_arena_2);
			inter_a3 = find_intersection(c.a3, pt_arena_1, pt_arena_2);

			if (inter_a1 || inter_a2 || inter_a3)
			{
				intersection = true;
			}

			pt_arena_1 = pt_arena_1->pnext;
		}

		/* Search an intersection between the arcs of the dubins curve and all the obstacles */
		polygon *obs = obstacle;
		while (obs != NULL && intersection == false)
		{
			point_node *pt_ob_1 = obs->pl->head;
			while (pt_ob_1 != NULL && intersection == false)
			{
				point_node *pt_ob_2;
				if (pt_ob_1->pnext == NULL)
				{
					pt_ob_2 = obs->pl->head;
				}
				else
				{
					pt_ob_2 = pt_ob_1->pnext;
				}

				inter_a1 = find_intersection(c.a1, pt_ob_1, pt_ob_2);
				inter_a2 = find_intersection(c.a2, pt_ob_1, pt_ob_2);
				inter_a3 = find_intersection(c.a3, pt_ob_1, pt_ob_2);

				if (inter_a1 || inter_a2 || inter_a3)
				{
					intersection = true;
				}
				pt_ob_1 = pt_ob_1->pnext;
			}
			obs = obs->pnext;
		}
	}
	if (intersection)
	{
		return make_tuple(c, 0);
	}
	return make_tuple(c, pidx);
}

/**
 * \fun
 * This function search the intersection points between an arc and a line.
 *
 * @param a: arc.
 * @param pnt: point_node*. First point of the line.
 * @param pnt_next: point_node*. Second point of the line.
 * @return bool. True if intersection is found, False otherwise.
 */
bool find_intersection(arc a, point_node *pnt, point_node *pnt_next)
{
	/*
		Given an arc and a line we search an intersection
		using the function intersCricleLine
	*/
	point_list *pts = NULL;
	double_list *p = NULL;
	tie(pts, p) = intersCircleLine(a.xc, a.yc, a.r,
								   pnt->x, pnt->y, pnt_next->x,
								   pnt_next->y);
	if (pts == NULL)
	{
		return false;
	}

	// Check if the intersection point are included in the dubin arc
	else
	{
		if (pt_in_arc(pts->head, a) == false)
		{
			double th0, thf;
			th0 = get_angle(a.xc, a.yc, a.x0, a.y0);
			thf = get_angle(a.xc, a.yc, a.xf, a.yf);

			if (a.th0 > a.thf)
			{
				double tmp = th0;
				th0 = thf;
				thf = tmp;
			}

			double dx = pnt_next->x - pnt->x;
			double dy = pnt_next->y - pnt->y;
			double al = atan(dx / dy);

			if (al < th0 || al > thf)
			{
				return false;
			}
		}
	}

	return true;
}
/**
 * \fun
 * This function check if a point is part of a given arc.
 * To do this, the angle formed in the circumference by the starting point is taken,
 * that formed by the end point and that formed by the point to be checked. If this
 * angle is included between the starting and ending angle then it is part of the arc.
 *
 * @param ptso: point_node *. Point to check.
 * @param a: arc.
 * @return bool. True if the point is in the arc. False otherwise.
 */
bool pt_in_arc(point_node *ptso, arc a)
{
	/*
		If an intersection is found it's necessary to make
		sure that the point is not part of the arc of the curve
	*/
	point_node *pts = ptso;
	double angle, start, end;
	// Get the circonference angle of given point in it
	start = get_angle(a.xc, a.yc, a.x0, a.y0);
	end = get_angle(a.xc, a.yc, a.xf, a.yf);

	if (end < start)
	{
		double tmp;
		tmp = end;
		end = start;
		start = tmp;
	}

	while (pts != NULL)
	{
		angle = get_angle(a.xc, a.yc, pts->x, pts->y);
		if (is_in_arc(start, end, angle))
		{
			return true;
		}
		pts = pts->pnext;
	}
	return false;
}

/**
 * \fun
 * This function convert the given arc in a Pose object for the simulator.
 *
 * @param a: arc. Given arc
 * @return Pose
 */
Pose get_pose(arc a)
{
	Pose p;
	p.s = a.L;
	p.kappa = a.k;

	p.x = a.x0;
	p.y = a.y0;
	p.theta = a.th0;

	return p;
}

/**
 * \fun
 * This function push the Pose in a Path vector that the simulator will use to move the robots.
 * For a better movement of the robors the dubins curve is sampled in n_samples subarc.
 *
 * @param c: curve. Given dubins curve.
 * @param p: Path. Path where to push the curve.
 * @return Path
 */
Path push_path(curve c, Path p)
{
	/*
	 * We sample each arc of the curve so as to have many smaller
	 * arcs and have more points of the path to pass to the simulator.
	 */

	arc a;
	int n_samples = 1000;
	a.x0 = c.a1.x0;
	a.y0 = c.a1.y0;
	a.th0 = c.a1.th0;
	a.k = c.a1.k;
	a.L = c.a1.L / n_samples;
	tie(a.xf, a.yf, a.thf, a.xc, a.yc, a.r) = circline(a.L, a.x0, a.y0, a.k, a.th0, false);
	p.points.push_back(get_pose(a));
	for (int i = 0; i < n_samples; i++)
	{

		a.x0 = a.xf;
		a.y0 = a.yf;
		a.th0 = a.thf;
		tie(a.xf, a.yf, a.thf, a.xc, a.yc, a.r) = circline(a.L, a.x0, a.y0, a.k, a.th0, false);
		p.points.push_back(get_pose(a));
	}

	a.x0 = c.a2.x0;
	a.y0 = c.a2.y0;
	a.th0 = c.a2.th0;
	a.k = c.a2.k;
	a.L = c.a2.L / n_samples;
	// p.points.push_back(get_pose(a));
	for (int i = 0; i < n_samples; i++)
	{

		a.x0 = a.xf;
		a.y0 = a.yf;
		a.th0 = a.thf;
		tie(a.xf, a.yf, a.thf, a.xc, a.yc, a.r) = circline(a.L, a.x0, a.y0, a.k, a.th0, false);
		p.points.push_back(get_pose(a));
	}

	a.x0 = c.a3.x0;
	a.y0 = c.a3.y0;
	a.th0 = c.a3.th0;
	a.k = c.a3.k;
	a.L = c.a3.L / n_samples;
	// p.points.push_back(get_pose(a));
	for (int i = 0; i < n_samples; i++)
	{

		a.x0 = a.xf;
		a.y0 = a.yf;
		a.th0 = a.thf;
		tie(a.xf, a.yf, a.thf, a.xc, a.yc, a.r) = circline(a.L, a.x0, a.y0, a.k, a.th0, false);
		p.points.push_back(get_pose(a));
	}

	return p;
}

tuple<vector<double>, vector<vector<curve>>> get_dubins_path_recursive(vector<double> x_path, vector<double> y_path, vector<double> th_path,
																	   points_map arena, bool inside_offset_arena, bool inside_offset_ob, double search_angle, bool first_iteration)
{
	curve c;
	int pidx;

	// These vectors are ordered. So at the first path position I have
	// a combination of curves and in the first length position it's total
	// length.
	vector<double> length;
	vector<vector<curve>> path;

	vector<double> t_length;
	vector<vector<curve>> t_path;

	// Exit condition for the ricorsion. If there are less than 2 points
	// Then there are no dubins path to calculate.
	if (x_path.size() < 2)
	{
		// Return 2 empty vectors.
		return make_tuple(length, path);
	}

	// Create sub-arrays of the path without the first element.
	vector<double> sub_x(x_path.begin() + 1, x_path.end());
	vector<double> sub_y(y_path.begin() + 1, y_path.end());
	vector<double> sub_th(th_path.begin() + 1, th_path.end());

	// Recursive call. The dubins path is calculated backward.
	tie(t_length, t_path) = get_dubins_path_recursive(sub_x, sub_y, sub_th, arena, inside_offset_arena, inside_offset_ob, search_angle, false);
	cout << "Ricevuto percorso di size: " << t_path.size() << endl;

	int cnt = 0;
	const int N_MAX_CURVES = 100;

	if (first_iteration == true)
	{
		point_list *map = arena.shrinked_arena;
		polygon *obs = arena.obstacles->offset_head;
		if (inside_offset_arena == true)
			map = arena.arena;
		if (inside_offset_ob == true)
			obs = arena.obstacles->head;

		if (x_path.size() == 2)
		{
			vector<double> thf_discretized = theta_discretization(th_path[1], search_angle);
			for (int i = 0; i < thf_discretized.size(); i++)
			{
				tie(c, pidx) = dubins_no_inter(x_path[0], y_path[0], th_path[0], x_path[1], y_path[1],
											   thf_discretized[i], map, obs);
				if (pidx > 0)
				{
					vector<curve> lpath;
					lpath.push_back(c);
					double l = c.L;

					if (cnt < N_MAX_CURVES)
					{
						int k = 0;
						bool insert = false;
						while (k < path.size() && insert == false)
						{
							if (l > length[k])
							{
								path.insert(path.begin() + k, lpath);
								length.insert(length.begin() + k, l);
								insert = true;
							}
							k++;
						}
						if (insert == false)
						{
							path.push_back(lpath);
							length.push_back(l);
						}
						cnt++;
					}
					else
					{
						int k = 0;
						bool insert = false;
						while (k < path.size() && insert == false)
						{
							if (l < length[k])
							{
								path.insert(path.begin() + k, lpath);
								length.insert(length.begin() + k, l);
								path.pop_back();
								length.pop_back();
								insert = true;
							}
							k++;
						}
					}
				}
			}
		}
		else
		{
			for (int i = 0; i < t_path.size(); i++)
			{
				double th = t_path[i][0].a1.th0;
				tie(c, pidx) = dubins_no_inter(x_path[0], y_path[0], th_path[0], x_path[1], y_path[1],
											   th, map, obs);
				if (pidx > 0)
				{

					vector<curve> lpath = t_path[i];
					lpath.insert(lpath.begin(), c);
					double l = t_length[i] + c.L;

					if (cnt < N_MAX_CURVES)
					{
						int k = 0;
						bool insert = false;
						while (k < path.size() && insert == false)
						{
							if (l > length[k])
							{
								path.insert(path.begin() + k, lpath);
								length.insert(length.begin() + k, l);
								insert = true;
							}
							k++;
						}
						if (insert == false)
						{
							path.push_back(lpath);
							length.push_back(l);
						}
						cnt++;
					}
					else
					{
						int k = 0;
						bool insert = false;
						while (k < path.size() && insert == false)
						{
							if (l < length[k])
							{
								path.insert(path.begin() + k, lpath);
								length.insert(length.begin() + k, l);
								path.pop_back();
								length.pop_back();
								insert = true;
							}
							k++;
						}
					}
				}
			}
		}
	}
	else
	{
		if (x_path.size() == 2)
		{
			vector<double> th0_discretized = theta_discretization(th_path[0], search_angle);
			vector<double> thf_discretized = theta_discretization(th_path[1], search_angle);

			for (int i = 0; i < thf_discretized.size(); i++)
			{
				for (int j = 0; j < th0_discretized.size(); j++)
				{
					point_list *map = arena.arena;

					tie(c, pidx) = dubins_no_inter(x_path[0], y_path[0], th0_discretized[j], x_path[1], y_path[1],
												   thf_discretized[i], map, arena.obstacles->offset_head);

					if (pidx > 0)
					{
						vector<curve> lpath;
						lpath.push_back(c);
						double l = c.L;

						if (cnt < N_MAX_CURVES)
						{
							int k = 0;
							bool insert = false;
							while (k < path.size() && insert == false)
							{
								if (l > length[k])
								{
									path.insert(path.begin() + k, lpath);
									length.insert(length.begin() + k, l);
									insert = true;
								}
								k++;
							}
							if (insert == false)
							{
								path.push_back(lpath);
								length.push_back(l);
							}
							cnt++;
						}
						else
						{
							int k = 0;
							bool insert = false;
							while (k < path.size() && insert == false)
							{
								if (l < length[k])
								{
									path.insert(path.begin() + k, lpath);
									length.insert(length.begin() + k, l);
									path.pop_back();
									length.pop_back();
									insert = true;
								}
								k++;
							}
						}
					}
				}
			}
		}
		else
		{
			vector<double> th0_discretized = theta_discretization(th_path[0], search_angle);

			for (int i = 0; i < t_path.size(); i++)
			{
				curve c_min;
				double pidx_min = 0;
				for (int j = 0; j < th0_discretized.size(); j++)
				{
					double thf = t_path[i][0].a1.th0;
					tie(c, pidx) = dubins_no_inter(x_path[0], y_path[0], th0_discretized[j], x_path[1], y_path[1],
												   thf, arena.shrinked_arena, arena.obstacles->offset_head);

					point_node *p1 = new point_node(x_path[0], y_path[0]);
					point_node *p2 = new point_node(x_path[1], y_path[1]);
					double distance = p1->distance(p2);
					if (pidx > 0)
					{
						vector<curve> lpath = t_path[i];
						lpath.insert(lpath.begin(), c);
						double l = t_length[i] + c.L;

						if (cnt < N_MAX_CURVES)
						{
							int k = 0;
							bool insert = false;
							while (k < path.size() && insert == false)
							{
								if (l > length[k])
								{
									path.insert(path.begin() + k, lpath);
									length.insert(length.begin() + k, l);
									insert = true;
								}
								k++;
							}
							if (insert == false)
							{
								path.push_back(lpath);
								length.push_back(l);
							}
							cnt++;
						}
						else
						{
							int k = 0;
							bool insert = false;
							while (k < path.size() && insert == false)
							{
								if (l < length[k])
								{
									path.insert(path.begin() + k, lpath);
									length.insert(length.begin() + k, l);
									path.pop_back();
									length.pop_back();
									insert = true;
								}
								k++;
							}
						}
					}
				}
			}
		}
	}
	return make_tuple(length, path);
}

tuple<vector<double>, vector<double>> refine_path(vector<double> x_path, vector<double> y_path)
{
	vector<double> new_x_path;
	vector<double> new_y_path;

	int size = x_path.size();

	double distance = 0;

	if (size > 0)
	{
		new_x_path.push_back(x_path[0]);
		new_y_path.push_back(y_path[0]);
		for (int i = 1; i < size; i++)
		{
			distance += sqrt(pow(x_path[i] - x_path[i - 1], 2) + pow(y_path[i] - y_path[i - 1], 2));
			if (distance > 0.05 || i == size - 1)
			{
				new_x_path.push_back(x_path[i]);
				new_y_path.push_back(y_path[i]);
				distance = 0;
			}
		}
	}
	return make_tuple(new_x_path, new_y_path);
}

/**
 * \fun
 * This function return the dubins path formed by all the dubins curve that a robot must follow.
 * Everytime a dubins curve is computed a copy of the arriving theta is stored in a vector.
 * This is useful in the case that if during the process no dubins curve is founded the function delete
 * the last curve computed and search another way to arrive to the next point. To do that is important
 * to avoid reusing arriving theta that we already know lead to dead ends.
 * In the case no dubins path is founded one possibly way to solve the problem is to increase the kmax value.
 * In this way a robot can perform tighter curves and it is easier to find a way forward but of course there
 * are physical limits to the robot on how tight it can turn.
 * Another way could be to reduce the offset of the arena and obstacles to make the passages wider.
 * In doing so, however, there is a risk that the robot could suffer a collision
 *
 * @param arena: points_map. Representation of the arena.
 * @param abstract_arena: Wordl_representation. Abstract representation of the arena.
 * @param r: Robot *. Robot for which we need to calculate the dubins path
 * @return vector<curve>: Dubins path.
 */
vector<curve> get_dubins_path(points_map arena, World_representation abstract_arena, Robot *r)
{
	/* centroid path coordinates vectors */
	vector<double> x_path;
	vector<double> y_path;
	vector<double> th_path; // The angles are in radiants!

	/* Get the cells centroids of the path */
	tie(x_path, y_path) = abstract_arena.get_path(r->plan, arena.obstacles->offset_head);

	if (x_path.size() == 1)
	{
		x_path.insert(x_path.begin(), r->location->x);
		y_path.insert(y_path.begin(), r->location->y);
	}
	else
	{
		x_path[0] = r->location->x;
		y_path[0] = r->location->y;
	}

	tie(x_path, y_path) = refine_path(x_path, y_path);
	/* Get the starting angle for moving from a cell to another one */
	th_path = opti_theta(x_path, y_path);

	/* Space where to search a minimum dubins curve */
	double search_angle = M_PI;

	vector<double> all_lengths;
	vector<vector<curve>> all_paths;

	th_path[0] = r->theta;

	tie(all_lengths, all_paths) = get_dubins_path_recursive(x_path, y_path, th_path, arena,
															r->inside_offset_arena,
															r->inside_offset_obstacle,
															search_angle, true);

	double length;
	vector<curve> path;

	for (int i = 0; i < all_paths.size(); i++)
	{
		if ((all_paths[i].size() + 1) == x_path.size())
		{
			if (path.size() == 0)
			{
				length = all_lengths[i];
				path = all_paths[i];
			}
			else if (all_lengths[i] < length)
			{
				length = all_lengths[i];
				path = all_paths[i];
			}
		}
	}
	cout << "Fine" << endl;

	return path;
}

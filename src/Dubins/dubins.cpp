#include "dubins.h"

/*--------------------START DUBINS.H----------------------------------*/
tuple<int, curve> dubins(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax)
{
	double sc_th0, sc_thf, sc_Kmax, lambda;
	tie(sc_th0, sc_thf, sc_Kmax, lambda) = scaleToStandard(x0, y0, th0, xf, yf, thf, Kmax);
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
						ksigns[pidx][0] * Kmax, ksigns[pidx][1] * Kmax,
						ksigns[pidx][2] * Kmax);
		assert(check(sc_s1, ksigns[pidx][0] * sc_Kmax, sc_s2,
					 ksigns[pidx][1] * sc_Kmax, sc_s3,
					 ksigns[pidx][2] * sc_Kmax, sc_th0, sc_thf));
	}
	return make_tuple(pidx, c);
}

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

curve dubinscurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2)
{
	curve d;
	d.a1 = dubinsarc(x0, y0, th0, k0, s1);
	d.a2 = dubinsarc(d.a1.xf, d.a1.yf, d.a1.thf, k1, s2);
	d.a3 = dubinsarc(d.a2.xf, d.a2.yf, d.a2.thf, k2, s3);
	d.L = d.a1.L + d.a2.L + d.a3.L;
	return d;
}

tuple<double, double, double, double> scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax)
{
	double dx = xf - x0;
	double dy = yf - y0;
	double phi = atan2(dy, dx);
	double lambda = hypot(dx, dy) / 2;
	double sc_Kmax = Kmax * lambda;
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

tuple<double, double, double, double, double, double> circline(double s, double x0, double y0, double k, double th0, bool get_center)
{
	double x, y, th, xc, yc, r;
	x = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2);
	y = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s / 2);
	th = mod2pi(th0 + k * s);

	// Finding a third point included in the arc.
	// It's necessary to find the center of the circle
	double x1, y1;
	double s1 = s * 4;
	x1 = x0 + s1 * sinc(k * s1 / 2.0) * cos(th0 + k * s1 / 2);
	y1 = y0 + s1 * sinc(k * s1 / 2.0) * sin(th0 + k * s1 / 2);

	// aggiungere calcolo centro circonferenza
	if (get_center)
		tie(xc, yc, r) = get_circle_center(x0, y0, x1, y1, x, y);
	else
		xc = 0, yc = 0, r = 0;

	return make_tuple(x, y, th, xc, yc, r);
}

Mat plotdubins(curve d, string c1, string c2, string c3, Mat arena)
{
	arena = plotarc(d.a1, c1, arena);
	arena = plotarc(d.a2, c2, arena);
	arena = plotarc(d.a3, c3, arena);

	return arena;
}

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
		r = 255;
		g = 0;
		b = 0;
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

	img = plot_points(pl, img, Scalar(r, g, b), false);
	pl->delete_list();
	return img;
}

tuple<point_list *, double_list *> intersLineLine(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
{
	point_list *pts = new point_list;
	double_list *ts = new double_list;
	double minX1, minY1, maxX1, maxY1, minX2, minY2, maxX2, maxY2;
	double t;

	minX1 = min(x1, x2);
	minY1 = min(y1, y2);
	maxX1 = max(x1, x2);
	maxY1 = max(y1, y2);

	minX2 = min(x3, x4);
	minY2 = min(y3, y4);
	maxX2 = max(x3, x4);
	maxY2 = max(y3, y4);

	if (maxX2 < minX1 || minX2 > maxX1 || maxY2 < minY1 || minY2 > maxY1)
	{
		double_list *dnull = NULL;
		point_list *pnull = NULL;
		return make_tuple(pnull, dnull);
	}

	double q[] = {x1, y1};
	double s[] = {x2, y2};
	for (int i = 0; i < 2; i++)
		s[i] -= q[i];

	double p[] = {x3, y3};
	double r[] = {x2, y2};
	for (int i = 0; i < 2; i++)
		r[i] -= p[i];

	double diffPQ[2];
	for (int i = 0; i < 2; i++)
		diffPQ[i] = q[i] - p[i];

	double crossRS = cross2D(r, s);
	double crossDiffR = cross2D(diffPQ, r);
	double crossDiffS = cross2D(diffPQ, s);

	if (crossRS == 0)
	{
		double dotRR = dot2D(r, r);
		double dotSR = dot2D(s, r);
		double t0 = dot2D(diffPQ, r) / dotRR;
		double t1 = t0 + dotSR / dotRR;
		if (dotSR < 0)
		{
			if (t0 >= 0 && t1 <= 1)
			{
				ts->add_node(new double_node(max(t1, 0.0)));
				ts->add_node(new double_node(min(t0, 1.0)));
			}
		}
		else
		{
			if (t1 >= 0 && t0 <= 1)
			{
				ts->add_node(new double_node(max(t0, 0.0)));
				ts->add_node(new double_node(min(t1, 1.0)));
			}
		}
	}
	else
	{
		if (crossRS == 0 && crossDiffR != 0)
		{
			double_list *dnull = NULL;
			point_list *pnull = NULL;
			return make_tuple(pnull, dnull);
		}
		else
			t = crossDiffS / crossRS;
		double u = crossDiffR / crossRS;
		if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
			ts->add_node(new double_node(t));
	}
	double_node *tmp = ts->head;
	while (tmp != NULL)
	{
		pts->add_node(new point_node(p[0] + tmp->value * r[0], p[1] + tmp->value * r[1]));
		tmp = tmp->pnext;
	}

	return make_tuple(pts, ts);
}

double cross2D(double *v1, double *v2)
{
	return v1[0] * v2[1] - v1[1] * v2[0];
}

double dot2D(double *v1, double *v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1];
}

/*a,b = coordinate centro circonferenza arco.*/
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

vector<double> opti_theta(vector<double> xpath, vector<double> ypath)
{
	vector<double> thpath;
	if (xpath.size() > 0)
	{
		for (int i = 0; i < xpath.size() - 1; i++)
		{
			thpath.push_back(get_angle(xpath[i], ypath[i],
									   xpath[i + 1], ypath[i + 1]));
		}
		thpath.push_back(thpath[thpath.size() - 1]);
	}
	return thpath;
}

/*
	Source of this function:
	https://www.geeksforgeeks.org/equation-of-circle-when-three-points-on-the-circle-are-given/
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

vector<double> theta_discretization(double starting_angle, double search_angle)
{

	vector<double> plausible_theta;
	plausible_theta.push_back(starting_angle);
	double res_steps = M_PI / 180; // 1 degree converted in radiants

	double theta = starting_angle - (search_angle / 2);

	while (theta <= starting_angle + (search_angle / 2))
	{
		plausible_theta.push_back(theta);
		theta += res_steps;
	}
	return plausible_theta;
}

tuple<curve, int> dubins_no_inter(double x0, double y0, double th0,
								  double xf, double yf, double *thf,
								  double Kmax, point_list *arena_limits, polygon *obstacle,
								  double search_angle, vector<double> used_th)
{
	curve c, c_min;
	int pidx, min_pidx = 0;
	double initial_th = *thf;

	bool has_c_min = false;
	bool intersection = false;
	bool finished_angles = false;

	vector<double> angles = theta_discretization(initial_th, search_angle);

	// cout <<"[";
	// for(int i = 0;i<used_th.size();i++)
	// {
	// 	cout << used_th[i]<<",";
	// }
	// cout <<"]\n\n";
	int i = 0;
	/* Compute dubins path for all the angles available*/
	while (i < angles.size())
	{
		tie(pidx, c) = dubins(x0, y0, th0, xf, yf, angles[i], Kmax);
		intersection = false;
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

			/* If an intersection is not found the dubins curve is admitted*/
			if (intersection == false && count(used_th.begin(),used_th.end(),angles[i]) == 0)
			{
				/* Store the first curve found or sobstitute it if it's length is less*/
				if (has_c_min == false || c.L < c_min.L)
				{
					c_min = c;
					has_c_min = true;
					*thf = angles[i];
					min_pidx = pidx;
				}
			}
		}
		i++;

		/* If no curves is found, take the opposite searh area */
		if (i == angles.size() && min_pidx <= 0 && finished_angles == false)
		{
			finished_angles = true;
			initial_th = initial_th + M_PI;
			search_angle = (2 * M_PI) - search_angle;
			angles.clear();
			angles = theta_discretization(initial_th, search_angle);
			i = 0;
		}
	}	
	return make_tuple(c_min, min_pidx);
}

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
			return false;
		}
	}

	return true;
}

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

	/* It's simpler work with only positive angles*/
	if (start < 0)
		start += (M_PI * 2);
	if (end < 0)
		end += (M_PI * 2);

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
		if (angle < 0)
			angle += (M_PI * 2);
		if (is_in_arc(start, end, angle))
		{
			return true;
		}
		pts = pts->pnext;
	}
	return false;
}

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

Path push_path(curve c, Path p)
{
	/*
	 * We sample each arc of the curve so as to have many smaller
	 * arcs and have more points of the path to pass to the simulator.
	 */
	arc a;
	int n_samples = 1000;

	a = dubinsarc(c.a1.x0, c.a1.y0, c.a1.th0, c.a1.k, c.a1.L / n_samples);
	for (int i = 0; i < n_samples; i++)
	{
		p.points.push_back(get_pose(a));
		a = dubinsarc(a.xf, a.yf, a.thf, a.k, a.L);
	}

	a = dubinsarc(c.a2.x0, c.a2.y0, c.a2.th0, c.a2.k, c.a2.L / n_samples);
	for (int i = 0; i < n_samples; i++)
	{
		p.points.push_back(get_pose(a));
		a = dubinsarc(a.xf, a.yf, a.thf, a.k, a.L);
	}

	a = dubinsarc(c.a3.x0, c.a3.y0, c.a3.th0, c.a3.k, c.a3.L / n_samples);
	for (int i = 0; i < n_samples; i++)
	{
		p.points.push_back(get_pose(a));
		a = dubinsarc(a.xf, a.yf, a.thf, a.k, a.L);
	}
	p.points.push_back(get_pose(a));

	return p;
}

vector<curve> get_dubins_path(points_map arena, World_representation abstract_arena, Robot *r)
{
	vector<curve> path;

	/* centroid path coordinates vectors */
	vector<double> x_path;
	vector<double> y_path;
	vector<double> th_path; // The angles are in radiants!

	/* Get the cells centroids of the path */
	tie(x_path, y_path) = abstract_arena.get_path(r->plan);

	/* Get the starting angle for moving from a cell to another one */
	th_path = opti_theta(x_path, y_path);

	/* Overwrite first cell centroid's coordinates with robot's coordinates */
	if (x_path.size() != 0)
	{
		x_path[0] = r->location->x;
		y_path[0] = r->location->y;
		th_path[0] = r->theta;
	}

	curve c;
	int pidx;

	/* Copy of the theta's vector */
	vector<double> original_theta = th_path;

	/* Vector that stores the thetas used for all the dubins curve found */
	vector<double> used_theta[th_path.size()];

	// double kmax = 27;
	double kmax = 600;
	/* Space where to search a minimum dubins curve */
	double search_angle = M_PI / 2;

	int i = 0;
	int size = x_path.size();

	bool inside_offset_arena = r->inside_offset_arena;
	bool inside_offset_obstacle = r->inside_offset_obstacle;

	/* Calculate dubin's curves without intersection */
	while (i < size - 1)
	{
		point_list *tmp_arena_limits = arena.shrinked_arena;
		polygon *tmp_obstacle = arena.obstacles->offset_head;

		if (r->inside_offset_arena)
		{
			tmp_arena_limits = arena.arena;
			inside_offset_arena = false;
		}
		if (r->inside_offset_obstacle)
		{
			tmp_obstacle = arena.obstacles->head;
			inside_offset_obstacle = false;
		}

		tie(c, pidx) = dubins_no_inter(x_path[i], y_path[i], th_path[i], x_path[i + 1],
									   y_path[i + 1], &th_path[i + 1], kmax, tmp_arena_limits,
									   tmp_obstacle, search_angle, used_theta[i]);

		/* If pidx > 0 a curve is found */
		if (pidx > 0)
		{
			/* Store the curve found and the arrival theta used for that */
			path.push_back(c);
			used_theta[i].push_back(th_path[i + 1]);
			i++;
		}
		else
		{
			if (i > 0)
			{
				/* If no curve is found the last curve is removed to compute
				 * another one with a different arrival angle. */
				path.pop_back();

				/* Restore the original theta. In this way we avoid to use
				 * different angles than the previous ones */
				th_path[i] = original_theta[i];
				i--;

				if (i == 0)
				{
					inside_offset_arena = r->inside_offset_arena;
					inside_offset_obstacle = r->inside_offset_obstacle;
				}
			}
			else
			{
				/*
				 * Aggiungere al report che se kmax è troppo piccolo rischiamo di avere curve troppo larghe
				 * e potrebbe capitare di non riuscire a trovare un percorso continuo per il robot. Se dovesse
				 * capitare un' idea potrebbe essere quella di incrementare kmax per permettere curve più strette
				 * (rischiando di farlo sterzare male) oppure di ridurre l'offset(rischiando schianti)...
				 *  O entrambi per un divertimento maggiore.
				 */
				throw std::logic_error("NO DUBINS PATH AVAILABLE - kmax too small\n");
				return path;
			}
		}
	}
	return path;
}

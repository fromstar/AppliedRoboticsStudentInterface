#include "utility.h"

int SCALE_1=100;
int SCALE_2=250;

void double_list::add_node(double_node *node)
{
	size++;
	if(head==NULL)
	{
		head = node;
		tail = head;
		return;
	}
	tail->pnext=node;
	tail = tail->pnext;
}


void double_list::print_list()
{
	double_node *tmp = head;
	while(tmp != NULL)
	{
		cout<<"Value: "<<tmp->value<<endl;
		tmp = tmp->pnext;
	}
}

void double_list::delete_list()
{
	double_node *tmp;
	while(head != NULL)
	{
		tmp=head;
		head=head->pnext;
		delete tmp;
	}	
	size = 0;
}

void point_list::add_node(point_node *node)
{
	size++;
	if(head==NULL)
	{
		head = node;
		tail = head;
		x_min = node->x;
		x_max = x_min;
		y_min = node->y;
		y_max = y_min;
		return;
	}
	if(node->x < x_min)
		x_min = node->x;
	else if(node->x > x_max)
		x_max = node->x;
	if(node->y < y_min)
		y_min = node->y;
	else if(node->y > y_max)
		y_max = node->y;
		
	tail->pnext = node;
	tail = tail->pnext;
}

void point_list::print_list()
{
	point_node *tmp = head;
	int i = 0;
	while(tmp != NULL)
	{
		i++;
		cout << "Point: " << i << endl;
		cout<<"x: "<<tmp->x<<"\ny: "<<tmp->y<<endl<<endl;
		tmp = tmp->pnext;
	}
}

void point_list::delete_list()
{
	point_node *tmp;
	while(head != NULL)
	{
		tmp=head;
		head=head->pnext;
		delete tmp;
	}
	size = 0;	
}


 Mat plot_points(point_list * pl, Mat arena, Scalar colorline, bool isPolygon)
 {
	 if(pl != NULL)
	 {
		point_node *n1 = pl->head;
		point_node *n2;
		while(n1 != NULL && n1->pnext != NULL)
		{
			n2 = n1->pnext;
			Point pt1((n1->x*SCALE_1)+SCALE_2,(n1->y*-SCALE_1)+SCALE_2);
			Point pt2((n2->x*SCALE_1)+SCALE_2,(n2->y*-SCALE_1)+SCALE_2);
			line(arena,pt1,pt2,colorline,1);
			/*cout<<"P1 x: " << n1->x<<" y: " <<n1->y <<endl;
			cout<<"P2 x: " << n2->x<<" y: " <<n2->y <<endl;
			cout<<"Line drawed\n";*/
			n1 = n1->pnext;
		}
		
		if (isPolygon) //close the polygon connecting the last point with the firts one
			line(arena,Point((n1->x*SCALE_1)+SCALE_2,(n1->y*-SCALE_1)+SCALE_2),Point((pl->head->x*SCALE_1)+SCALE_2,(pl->head->y*-SCALE_1)+SCALE_2),colorline,1);
	 }
	return arena;
 }


double sinc(double t)
{
	if(abs(t)<0.002)
		return 1-pow(t,2)/6*(1 - pow(t,2)/20);
	else
		return sin(t)/t;
}

double mod2pi(double ang)
{
	double out = ang;
	while(out < 0)
		out = out + 2*M_PI;
	while(out >= 2*M_PI)
		out = out - 2*M_PI;
	return out;
}

double rangeSymm(double ang)
{
	double out = ang;
	while(out <= -M_PI)
		out = out + 2*M_PI;
	while(out > M_PI)
		out = out - 2*M_PI;
	return out;	
}

bool check(double s1,double k0,double s2,double k1,double s3,double k2,double th0,double thf)
{
	double x0 = -1;
	double y0 = 0;
	double xf = 1;
	double yf = 0;
	double eq1,eq2,eq3;
	double Lpos;
	eq1 = x0 + s1 * sinc((1 / 2.)*k0*s1)*cos(th0 + (1 / 2.)*k0*s1) + s2 * sinc((1 / 2.)*k1*s2)*cos(th0 + k0 * s1 + (1 / 2.)*k1*s2) + s3 * sinc((1 / 2.)*k2*s3)*cos(th0 + k0 * s1 + k1 * s2 + (1 / 2.)*k2*s3) - xf;
	eq2 = y0 + s1 * sinc((1 / 2.)*k0*s1)*sin(th0 + (1 / 2.)*k0*s1) + s2 * sinc((1 / 2.)*k1*s2)*sin(th0 + k0 * s1 + (1 / 2.)*k1*s2) + s3 * sinc((1 / 2.)*k2*s3)*sin(th0 + k0 * s1 + k1 * s2 + (1 / 2.)*k2*s3) - yf;
	eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);
	Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);
	return (sqrt(eq1*eq1 + eq2*eq2 + eq3*eq3) < 1.e-10) && Lpos;
}

void sort(double_list *t, point_list *pts)
{
	double_node *dbfst = t->head;
	double_node *dbsnd;
	point_node *ptfst = pts->head;
	point_node *ptsnd;
	while(dbfst != NULL)
	{
		dbsnd = dbfst->pnext;
		ptsnd = ptfst->pnext;
		while(dbsnd != NULL)
		{
			if(dbfst->value > dbsnd->value)
			{
				double tmp = dbfst->value;
				dbfst->value = dbsnd->value;
				dbsnd->value = tmp;
				
				tmp = ptfst->x;
				ptfst->x = ptsnd->x;
				ptsnd->x = tmp;
				
				tmp = ptfst->y;
				ptfst->y=ptsnd->y;
				ptsnd->y=tmp;

			}
			dbsnd = dbsnd->pnext;
			ptsnd = ptsnd->pnext;
		}
		dbfst = dbfst->pnext;
		ptfst = ptfst->pnext;
	}
}

tuple <double,double> get_new_point(double m1, double m2, double q1, double q2)
{
	double x,y;
	if(m1 == 0 && m2 == std::numeric_limits<double>::infinity()) {
		x = q2;
		y = q1;
	}
	else if(m2 == 0 && m1 == std::numeric_limits<double>::infinity()){
		x = q1;
		y = q2;
	}
	else if(m1 == 0){
		// y = q1
		// y = m2x + q2 ->  (y - q2)/m2 = x
		y = q1;
		x = (y-q2)/m2;
	}
	else if(m2 == 0){
		y = q2;
		x = (y-q1)/m1;
	}
	else if(m1 == std::numeric_limits<double>::infinity()){
		x = q1;
		y = m2*x + q2;
	}
	else if(m2 == std::numeric_limits<double>::infinity()){
		x = q2;
		y = m1*x + q1;
	}
	else{
		x = (q1-q2)/(m2-m1);
		y = m1 * ((q1 - q2) / (m2 - m1)) + q1;
	}

	return make_tuple(x,y);
}

polygon * polygon::add_offset(double offset){

	/* The idea is to calculate the lines coefficients in order to have the 
	 * generic equations for them and then find where they intersecate */

	point_list * new_pol = new point_list;
	point_node * p1 = pl->head;
	point_node * p2 = p1->pnext;

	// A matrix that contains in x the angular coefficient and in y the ordinates value of the lines offsetted
	double 	coeff [pl->size][2];

	double x,y;
	double center_x = 0, center_y = 0;

	while (p1 != NULL)
	{
		center_x += p1->x;
		center_y += p1->y;
		p1=p1->pnext;
	}
	center_x = center_x/pl->size;
	center_y = center_y/pl->size;
	p1 = pl->head;

	// I have to find the equation for the parallel offsetted line
	for (int i=0;i<pl->size;i++)
	{
		double x1 = p1->x;
		double y1 = p1->y;

		double x2 = p2->x;
		double y2 = p2->y;

		// middle point coordinates
		double xm = (x2+x1)/2;
		double ym = (y2+y1)/2;

		double m,q;
		double qp1,qp2;

		double d1;
		double d2;
		// Particular cases
		if(x2 == x1)
		{
			m = std::numeric_limits<double>::infinity();
			// If m=inf the line is vertical -> x = k
			qp1 = xm + offset;
			qp2 = xm - offset;

			d1 = abs(qp1 - center_x);
			d2 = abs(qp2 - center_x);
		}
		else if(y2 == y1)
		{
			m = 0;
			// If m=0-> y = k. In this case k = y +- offset
			qp1 = ym + offset;
			qp2 = ym - offset;
			d1 = abs(qp1 - center_y);
			d2 = abs(qp2 - center_y);
		}
		// Generic case
		else{
			// https://www.youmath.it/formulari/formulari-di-geometria-analitica/430-formule-retta.html
			// Angular coefficient of the two lines
			m = (y2 - y1)/(x2 - x1);
			// Ordered at the origin
			q = ((x2*y1) - (x1*y2))/(x2-x1);

			// The parallel line has same m but different q.
			// Distance between two lines: d = |q1 - q2|/sqrt(m+1)
			// with q2 unknow. So i have 2 possible solutions for q2
			qp1 = y1 - m*x1 - offset*sqrt(1+pow(m,2));
			qp2 = offset*sqrt(pow(m,2)+1) + y1 - m*x1;

			d1 = abs(center_y - (m*center_x + qp1))/sqrt(1+pow(m,2));
			d2 = abs(center_y - (m*center_x + qp2))/sqrt(1+pow(m,2));

		}
		coeff[i][0] = m;
		// If the distance from the center of gravity of the firs lines is bigger than the second I have to take it.
		if (d1>d2){
			coeff[i][1] = qp1;
		}
		else{
			coeff[i][1] = qp2;
		}

		if(p2->pnext == NULL)
			p2 = pl->head;
		else
			p2 = p2->pnext;

		p1=p1->pnext;
	}
	// The interesection coordinates of two lines are:
	// x = (q1-q2)/(m2-m1); y = m1 * ((q1 - q2) / (m2 - m1)) + q1 

	for(int i=1;i<pl->size;i++)
	{	
		double m1 = coeff[i-1][0];
		double m2 = coeff[i][0];
		double q1 = coeff[i-1][1];
		double q2 = coeff[i][1];

		tie(x,y) = get_new_point(m1,m2,q1,q2);
		new_pol->add_node(new point_node(x,y));
	}
	tie(x,y) = get_new_point(coeff[pl->size-1][0], coeff[0][0], coeff[pl->size-1][1], coeff[0][1]);
	new_pol->add_node(new point_node(x,y));

	return new polygon(new_pol);
};
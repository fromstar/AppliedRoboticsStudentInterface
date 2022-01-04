#include "utility.h"

int SCALE_1=100;
int SCALE_2=150;

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
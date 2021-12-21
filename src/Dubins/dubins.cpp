#include "dubins.h"

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
		return;
	}
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

/*--------------------START DUBINS.H----------------------------------*/
tuple <int, curve> dubins(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax)
{
	double sc_th0, sc_thf, sc_Kmax, lambda;
	tie(sc_th0, sc_thf, sc_Kmax, lambda) = scaleToStandard(x0,y0,th0,xf,yf,thf,Kmax);
	int ksigns[6][3] = {{1,0,1},{-1,0,-1},{1,0,-1},{-1,0,1},{-1,1,-1},{1,-1,1}};
	int pidx = -1;
	double L = INFINITY;
	curve c;
	
	double sc_s1, sc_s2, sc_s3;
	double s1, s2, s3;
	bool ok;
	for(int i=0;i<6;i++)
	{
		double sc_s1_c, sc_s2_c, sc_s3_c;
		switch(i){
			case 0:
				tie(ok,sc_s1_c, sc_s2_c, sc_s3_c) = LSL(sc_th0, sc_thf, sc_Kmax);
			break;
			case 1:
				tie(ok,sc_s1_c, sc_s2_c, sc_s3_c) = RSR(sc_th0, sc_thf, sc_Kmax);
			break;
			case 2:
				tie(ok,sc_s1_c, sc_s2_c, sc_s3_c) = LSR(sc_th0, sc_thf, sc_Kmax);
			break;
			case 3:
				tie(ok,sc_s1_c, sc_s2_c, sc_s3_c) = RSL(sc_th0, sc_thf, sc_Kmax);
			break;
			case 4:
				tie(ok,sc_s1_c, sc_s2_c, sc_s3_c) = RLR(sc_th0, sc_thf, sc_Kmax);
			break;
			case 5:
				tie(ok,sc_s1_c, sc_s2_c, sc_s3_c) = LRL(sc_th0, sc_thf, sc_Kmax);
			break;
		}
		double Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
		if(ok && Lcur<L)
		{
			L = Lcur;
			sc_s1 = sc_s1_c;
			sc_s2 = sc_s2_c;
			sc_s3 = sc_s3_c;
			pidx = i;
		}
	}
	
	ok = false;
	if(pidx>0){
		ok = true;
		tie(s1,s2,s3) = scaleFromStandard(lambda,sc_s1,sc_s2,sc_s3);
		c = dubinscurve(x0,y0,th0,s1,s2,s3,ksigns[pidx][0]*Kmax, ksigns[pidx][1]*Kmax, ksigns[pidx][2]*Kmax);
		assert(check(sc_s1,ksigns[pidx][0]*sc_Kmax,sc_s2,ksigns[pidx][1]*sc_Kmax, sc_s3, ksigns[pidx][2]*sc_Kmax, sc_th0, sc_thf));
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
	tie(c.xf, c.yf, c.thf) = circline(L, x0, y0, k, th0);
	return c;
}

curve dubinscurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2)
{
	curve d;
	d.a1 = dubinsarc(x0,y0,th0,k0,s1);
	d.a2 = dubinsarc(d.a1.xf,d.a1.yf,d.a1.thf,k1,s2);
	d.a3 = dubinsarc(d.a2.xf,d.a2.yf,d.a2.thf,k2,s3);
	d.L = d.a1.L+d.a2.L+d.a3.L;
	return d;
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

tuple <double, double, double, double> scaleToStandard(double x0,double y0,double th0,double xf,double yf,double thf,double Kmax)
{
	double dx=xf-x0;
	double dy=yf-y0;
	double phi = atan2(dy,dx);
	double lambda = hypot(dx,dy)/2;
	double sc_Kmax = Kmax * lambda;
	double sc_th0 = mod2pi(th0-phi);
	double sc_thf = mod2pi(thf-phi);
	return make_tuple(sc_th0, sc_thf, sc_Kmax, lambda);
}

tuple <double, double, double> scaleFromStandard(double lambda,double sc_s1,double sc_s2,double sc_s3) 
{	// invert scaling transform (back to original data)
	double s1 = sc_s1 * lambda;
	double s2 = sc_s2 * lambda;
	double s3 = sc_s3 * lambda;
	return make_tuple(s1,s2,s3);
}

tuple <bool, double, double, double> LSL(double sc_th0,double sc_thf,double sc_Kmax) 
{
	double invK, C, S, temp1, temp2, sc_s1, sc_s2, sc_s3;
	bool ok;
	invK = 1 / sc_Kmax;
	C = cos(sc_thf) - cos(sc_th0);
	S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
	temp1 = atan2(C, S);
	sc_s1 = invK * mod2pi(temp1 - sc_th0);
    temp2 = 2+ 4*pow(sc_Kmax,2) -2*cos(sc_th0 - sc_thf) + 4 * sc_Kmax*(sin(sc_th0) - sin(sc_thf));
	if (temp2 < 0)
		return make_tuple(false,0,0,0);
		
	sc_s2 = invK * sqrt(temp2);
    sc_s3 = invK * mod2pi(sc_thf-temp1);
	ok = true;
	return make_tuple(ok, sc_s1, sc_s2, sc_s3);
}
tuple <bool, double, double, double> RSR(double sc_th0,double sc_thf,double sc_Kmax) 
{
	double invK, C, S, temp1, temp2, sc_s1, sc_s2, sc_s3;
	bool ok;
	invK = 1 / sc_Kmax;
	C = cos(sc_th0) - cos(sc_thf);
	S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
	temp1 = atan2(C, S);
	sc_s1 = invK * mod2pi(sc_th0-temp1);
    temp2 = 2+ 4*pow(sc_Kmax,2) -2*cos(sc_th0 - sc_thf) - 4 * sc_Kmax*(sin(sc_th0) - sin(sc_thf));
    if(temp2<0)
		return make_tuple(false,0,0,0);
	sc_s2 = invK * sqrt(temp2);
    sc_s3 = invK * mod2pi(temp1-sc_thf);
	ok = true;
	return make_tuple(ok, sc_s1, sc_s2, sc_s3);
}
tuple <bool, double, double, double> LSR(double sc_th0,double sc_thf,double sc_Kmax) 
{
	double invK, C, S, temp1, temp2, temp3, sc_s1, sc_s2, sc_s3;
	bool ok;
	 invK = 1 / sc_Kmax;
	C = cos(sc_th0) + cos(sc_thf);
	S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
	temp1 = atan2(-C, S);
    temp3 = 4*pow(sc_Kmax,2)-2 +2*cos(sc_th0 - sc_thf) + 4 * sc_Kmax*(sin(sc_th0) + sin(sc_thf));
	if(temp3<0)
		return make_tuple(false,0,0,0);
	sc_s2 = invK * sqrt(temp3);
    temp2 = -atan2(-2,sc_s2*sc_Kmax);   
	sc_s1 = invK * mod2pi(temp1+temp2-sc_th0);
    sc_s3 = invK * mod2pi(temp1 +temp2-sc_thf);
	ok = true;
	return make_tuple(ok, sc_s1, sc_s2, sc_s3);
}

tuple <bool, double, double, double> RSL(double sc_th0,double sc_thf,double sc_Kmax) 
{
	double invK, C, S, temp1, temp2, temp3, sc_s1, sc_s2, sc_s3;
	bool ok;
	invK = 1 / sc_Kmax;
	C = cos(sc_th0) + cos(sc_thf);
	S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
	temp1 = atan2(C, S);
    temp3 = 4*pow(sc_Kmax,2)-2 +2*cos(sc_th0 - sc_thf) - 4 * sc_Kmax*(sin(sc_th0) + sin(sc_thf));
    if (temp3 < 0)
		return make_tuple(false,0,0,0);
	sc_s2 = invK * sqrt(temp3);
    temp2 = atan2(2,sc_s2*sc_Kmax);   
	sc_s1 = invK * mod2pi(sc_th0-temp1+temp2);
    sc_s3 = invK * mod2pi(sc_thf-temp1+temp2);
	ok = true;
	return make_tuple(ok, sc_s1, sc_s2, sc_s3);
}

tuple <bool, double, double, double> RLR(double sc_th0,double sc_thf,double sc_Kmax) 
{
	double invK, C, S, temp1, temp2, sc_s1, sc_s2, sc_s3;
	bool ok;
	invK = 1 / sc_Kmax;
	C = cos(sc_th0) - cos(sc_thf);
	S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
	temp1 = atan2(C, S);
    temp2 = 0.125*(6-4 * pow(sc_Kmax,2)  + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax*(sin(sc_th0) - sin(sc_thf)));
    if (abs(temp2) > 1)
		return make_tuple(false,0,0,0);
	sc_s2 = invK * mod2pi(2*M_PI - acos(temp2));
	sc_s1 = invK * mod2pi(sc_th0-temp1+0.5*sc_s2*sc_Kmax);
    sc_s3 = invK * mod2pi(sc_th0-sc_thf+sc_Kmax*(sc_s2-sc_s1));
	ok = true;
	return make_tuple(ok, sc_s1, sc_s2, sc_s3);
}

tuple <bool, double, double, double> LRL(double sc_th0,double sc_thf,double sc_Kmax) 
{
	double invK, C, S, temp1, temp2, sc_s1, sc_s2, sc_s3;
	bool ok;
	invK = 1 / sc_Kmax;
	C = cos(sc_thf) - cos(sc_th0);
	S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
	temp1 = atan2(C, S);
    temp2 = 0.125*(6-4 * pow(sc_Kmax,2)  + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax*(sin(sc_th0) - sin(sc_thf)));
    if (abs(temp2) > 1)
		return make_tuple(false,0,0,0);
	sc_s2 = invK * mod2pi(2*M_PI - acos(temp2));
	sc_s1 = invK * mod2pi(temp1-sc_th0+0.5*sc_s2*sc_Kmax);
    sc_s3 = invK * mod2pi(sc_thf-sc_th0+sc_Kmax*(sc_s2-sc_s1));
	ok = true;
	return make_tuple(ok, sc_s1, sc_s2, sc_s3);
}

tuple <double, double, double> circline(double s,double x0, double y0,double k,double th0)
{
	double x,y,th;
	x = x0 + s*sinc(k*s/2.0)*cos(th0+k*s/2);
	y = y0 + s*sinc(k*s/2.0)*sin(th0+k*s/2);
	th = mod2pi(th0 + k*s);
	return make_tuple(x,y,th);
}

void python_plot(curve d, string c1, string c2, string c3)
{
	string a1_values = to_string(d.a1.x0) + " " + to_string(d.a1.y0) + " " + to_string(d.a1.th0) + " " + to_string(d.a1.k) + " " + to_string(d.a1.L) + " " + to_string(d.a1.xf) + " " + to_string(d.a1.yf) + " " + to_string(d.a1.thf);
	string a2_values = to_string(d.a2.x0) + " " + to_string(d.a2.y0) + " " + to_string(d.a2.th0) + " " + to_string(d.a2.k) + " " + to_string(d.a2.L) + " " + to_string(d.a2.xf) + " " + to_string(d.a2.yf) + " " + to_string(d.a2.thf);
	string a3_values = to_string(d.a3.x0) + " " + to_string(d.a3.y0) + " " + to_string(d.a3.th0) + " " + to_string(d.a3.k) + " " + to_string(d.a3.L) + " " + to_string(d.a3.xf) + " " + to_string(d.a3.yf) + " " + to_string(d.a3.thf);
	
	string command = "python3 plot.py " + a1_values + " " + a2_values + " " + a3_values  + " " + c1 + " " + c2 + " " + c3;
	system((command).c_str()); 
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
	Point pts[npts];
	point_list *pl = new point_list;
	for(int i=0;i<npts;i++)
	{
		double x,y,s;
		s = a.L/npts*i;
		tie(x,y,th) = circline(s, a.x0, a.y0, a.k, a.th0);
		pl->add_node(new point_node(x,y));
	}
	
	int r=0,g=0,b=0;
	if(!strcmp(c.c_str(),"r") || !strcmp(c.c_str(),"red")) //strcmp() returns 0 if the strings are equal
	{
		r = 255;
		g = 0;
		b = 0;
	}
	else if (!strcmp(c.c_str(),"g") || !strcmp(c.c_str(),"green"))
	{
		r = 0;
		g = 255;
		b = 0;
	}
	else if (!strcmp(c.c_str(),"b") || !strcmp(c.c_str(),"blue"))
	{
		r = 0;
		g = 0;
		b = 255;
	}

	img = plot_points(pl,img,Scalar(r,g,b),false);
	pl->delete_list();
	return img;
}

tuple <point_list *, double_list *> intersLineLine(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
{
	point_list *pts = new point_list;
	double_list *ts = new double_list;
	double minX1, minY1, maxX1, maxY1, minX2, minY2, maxX2, maxY2;
	double t;
	
	minX1 = min(x1,x2);
	minY1 = min(y1,y2);
	maxX1 = max(x1,x2);
	maxY1 = max(y1,y2);
   
	minX2 = min(x3,x4);
	minY2 = min(y3,y4);
	maxX2 = max(x3,x4);
	maxY2 = max(y3,y4);
	
	if (maxX2<minX1 || minX2>maxX1 || maxY2<minY1 || minY2>maxY1){
		double_list *dnull=NULL;
		point_list *pnull=NULL;
		return make_tuple(pnull,dnull);}
	
	double q[] = {x1,y1};
	double s[] = {x2,y2};
	for(int i=0;i<2;i++)
		s[i]-=q[i];
		
	double p[] = {x3,y3};
	double r[] = {x2,y2};
	for(int i=0;i<2;i++)
		r[i]-=p[i];
	
	double diffPQ[2];
	for(int i=0;i<2;i++)
		diffPQ[i]=q[i]-p[i];
	
	double crossRS = cross2D(r,s);
	double crossDiffR = cross2D(diffPQ,r);
	double crossDiffS = cross2D(diffPQ,s);
	
	if(crossRS == 0)
	{
		double dotRR = dot2D(r,r);
		double dotSR = dot2D(s,r);
		double t0 = dot2D(diffPQ,r)/dotRR;
		double t1 = t0+dotSR/dotRR;
		if (dotSR<0)
		{
			if (t0 >= 0 && t1 <= 1)
			{
				ts->add_node(new double_node(max(t1,0.0))); 
				ts->add_node(new double_node(min(t0,1.0)));
			}
		}
		else
		{
			if (t1 >= 0 && t0 <= 1)
			{
				ts->add_node(new double_node(max(t0,0.0))); 
				ts->add_node(new double_node(min(t1,1.0)));
			}
		}
	}
	else
	{
		if(crossRS == 0 && crossDiffR != 0){
			double_list *dnull=NULL;
			point_list *pnull=NULL;
			return make_tuple(pnull,dnull);}
		else
			t = crossDiffS/crossRS;
			double u = crossDiffR/crossRS;
			 if (t>=0 && t<= 1 && u>=0 && u<=1)
				ts->add_node(new double_node(t));
	}
	double_node *tmp = ts->head;
	while(tmp!=NULL)
	{
		pts->add_node(new point_node(p[0]+tmp->value*r[0], p[1]+tmp->value*r[1]));
		tmp=tmp->pnext;
	}
	
	return make_tuple(pts,ts);
}

double cross2D(double *v1, double *v2)
{
	return v1[0]*v2[1]-v1[1]*v2[0];
}

double dot2D(double *v1,double *v2)
{
   return v1[0]*v2[0]+v1[1]*v2[1]; 
}

/*a,b = coordinate centro circonferenza arco.*/
tuple <point_list *, double_list *> intersCircleLine(double a, double b, double r, double x1, double y1, double x2, double y2)
{
	double p1,p2,p3,p4,p5,p6;
	p1 = 2*x1*x2;
    p2 = 2*y1*y2;
    p3 = 2*a*x1;
    p4 = 2*a*x2;
    p5 = 2*b*y1;
    p6 = 2*b*y2;
    
    double c1,c2,c3;
    c1 = pow(x1,2)+pow(x2,2)-p1+pow(y1,2)+pow(y2,2)-p2;
    c2 = -2*pow(x2,2)+p1-p3+p4-2*pow(y2,2)+p2-p5+p6;
    c3 = pow(x2,2)-p4+pow(a,2)+pow(y2,2)-p6+pow(b,2)-pow(r,2);
    
    double delta = pow(c2,2) - 4*c1*c3;
    point_list *pts = new point_list;
    double_list *t = new double_list;
    
    double t1,t2,x,y;
    double  deltaSq;
    if (delta<0){
		double_list *dnull = NULL;
		point_list *pnull = NULL;
		cout<<"DELTA: "<<delta<<endl;
		return make_tuple(pnull,dnull);}
	else
	{
		if(delta>0)
		{
			deltaSq = sqrt(delta);
			t1 = (-c2+deltaSq)/(2*c1);
			t2 = (-c2-deltaSq)/(2*c1);
		}
		else
		{
			t1 = -c2/(2*c1);
			t2 = t1;
		}
	}
	if (t1 >=0 && t1<=1)
	{
		x = x1*t1+x2*(1-t1);
		y = y1*t1+y2*(1-t1);
		Point pt(x,y);
		pts->add_node(new point_node(x,y));
		t->add_node(new double_node(t1));
		cout<<"NODO AGGIUNTO\n";
	}
	if (t2 >=0 && t2<=1 && t2!=t1)
	{
		x = x1*t2+x2*(1-t2);
		y = y1*t2+y2*(1-t2);
		Point pt(x,y);
		cout<<typeid(pt.x).name()<<endl;
		pts->add_node(new point_node(x,y));
		
		t->add_node(new double_node(t2));
		cout<<"NODO AGGIUNTO\n";
	}
	sort(t,pts);
	return make_tuple(pts,t);
 }
 
 Mat plot_points(point_list * pl, Mat arena, Scalar colorline, bool isPolygon)
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
	return arena;
 }


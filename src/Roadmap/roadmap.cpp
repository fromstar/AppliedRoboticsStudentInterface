#include "roadmap.h"

tuple <double,double> get_offsetted_coordinates(point_node *p1, point_node *p2, point_node *p3, double x_min, double x_max)
{
	double x1 = p1->x;
	double y1 = p1->y;

	double x2 = p2->x;
	double y2 = p2->y;

	double x3 = p3->x;
	double y3 = p3->y;
	// Angular coefficient of the two lines
	// https://www.youmath.it/formulari/formulari-di-geometria-analitica/430-formule-retta.html
	double m1 = (y1 - y2)/(x1 - x2);
	double m2 = (y2 - y3)/(x2 - x3);

	// Ordered at the origin
	double q1 = ((x1*y2) - (x2*y1))/(x1-x2);
	double q2 = ((x2*y3) - (x3*y2))/(x2-x3);

	// Bisector coefficient
	// https://meetheskilled.com/retta-con-coefficiente-angolare-noto/
	// https://www.youmath.it/formulari/formulari-di-geometria-analitica/434-equazione-della-retta-passante-per-due-punti.html
	double mb = ((m1 * sqrt(1 + pow(m2,2))) + (m2 * sqrt(1 + pow(m1,2))))/(sqrt(1 + pow(m2,2)) + sqrt(1 + pow(m1,2)));
	double qb = (mb*-1*x2)+y2;

	// General line equation: y = f(x) = mx + q.
	// The offsetted point has coordinate (x,f(x)) and the distance between 2 points is d = sqrt((x2-x1)^2 + (y2-y1)^2).
	// Call the coordinate for the polygon point A(ax,ay) and the offsetted point B(bx,by) with by = f(bx) = mb*x+qb. Distance d is our offset.
	// So: d^2 = (bx - ax)^2 + (mbx + q - ay)^2 = 
	// d^2 = bx^2 - 2bxax + ax^2 + m^2bx^2 + q^2 + ay^2 + 2mqbx - 2maybx - 2qay
	// d^2 = bx^2(m^2 + 1) bx(-2ax + 2mq -2may) + ax^2 + q^2 +ay^2 - 2qay
	// let's move d^2 to the left and call the coefficient A,B,C: Abx^2 + Bbx + C
	double A = pow(mb,2) + 1;
	double B = -2*x2 + 2*mb*qb - 2*mb*y2;
	double C = pow(x2,2)+pow(qb,2) + pow(y2,2) - 2*qb*y2;

	//offsettet x
	double ox1 = (-B - sqrt(pow(B,2)-(4*A*C)))/(2*A);
	double ox2 = (-B + sqrt(pow(B,2)-(4*A*C)))/(2*A);
	//offsetted y
	double oy;
	if(ox1 < x_min || ox1 > x_max)
		return make_tuple(ox1, mb*ox1 + qb);
	else
		return make_tuple(ox1, mb*ox2 + qb);

}

void polygon::add_offset(double offset){
	/*point_node * p1 = pl->head;
	point_node * p2 = p1->pnext;
	point_node * p3 = p2->pnext;
	double x,y;

	// I have to find the bisector of the angle formed by two lines.
	// These lines are formed connecting 2 points of the polygon
	while(p3 != NULL) 
	{
		// Update central point
		tie(x,y) = get_offsetted_coordinates(p1,p2,p3,pl->x_min,pl->x_max);
		p2->x = x;
		p2->y = y;

		p1=p1->pnext;
		p2=p2->pnext;
		p3=p3->pnext;
	}*/
};

void points_map::add_arena_points(point_list *ArenaPoints){
	arena = ArenaPoints;
};

void points_map::set_robot_position(double x, double y){
  robot = new(Robot);
  robot -> x = x;
  robot -> y = y;
};

void points_map::add_gate(polygon *gt){
	if(gates->head == NULL)
	{
		gates->head = gt;
		gates->tail = gates->head;
		return;
	}
	gates->tail->pnext = gt;
	gates->tail = gates->tail->pnext;
};

void points_map::add_obstacle(polygon *ob){
	ob->add_offset(obstacles->offset);
	if(obstacles->head == NULL)
	{
		obstacles->head = ob;
		obstacles->tail = obstacles->head;
		return;
	}
	obstacles->tail->pnext = ob;
	obstacles->tail = obstacles->tail->pnext;
};

void points_map::print_info(){
  cout<<"Robot location: " << robot->x << " - "<< robot->y <<endl;
};

Mat points_map::plot_arena(){
	Mat img_arena(600,600, CV_8UC3, Scalar(255, 255, 255));
	
	img_arena = plot_points(arena, img_arena, Scalar(0,0,0),true);
	
	polygon *tmp = obstacles->head;
	while(tmp != NULL)
	{
		img_arena = plot_points(tmp->pl,img_arena,Scalar(0,0,255),true);
		tmp = tmp->pnext;
	}
	
	tmp = gates->head;
	while(tmp != NULL)
	{
		img_arena = plot_points(tmp->pl,img_arena,Scalar(0,255,0),true);
		tmp = tmp->pnext;
	}
	
	return img_arena;
}

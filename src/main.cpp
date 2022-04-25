#include "Roadmap/roadmap.h"
//#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include <time.h>

using namespace std;

void test();
void test2();

int main(){
    //point_list * p = new point_list;
	test();
	return 0;
};

void test2(){
	point_node *p1 = new point_node(0, 0);
	point_node *p2 = new point_node(3, 0);
	point_node *p3 = new point_node(1, 1);

	point_node *d = new point_node(2, 1);
	point_node *e = new point_node(3, 1);
	point_node *f = new point_node(1, -1);
	
	point_list* pol_1 = new point_list;
	pol_1 -> add_node(p1);
	pol_1 -> add_node(p2);
	pol_1 -> add_node(p3);
	
	point_list *pol_2 = new point_list;
	pol_2 -> add_node(d);
	pol_2 -> add_node(e);
	pol_2 -> add_node(f);

	polygon *p = new polygon(pol_1);
	polygon *p_2 = new polygon(pol_2);
	
	points_map *map = new points_map;
	map -> add_obstacle(p);
	map -> add_obstacle(p_2);

	map -> merge_obstacles();

	Mat map_show = map -> plot_arena(800, 800, false);
	imshow("Test_2",map_show);
	waitKey(0);
};

void test()
{	
	printf("Code started\n");
	points_map test_map;
	
	// Define arena limits

	point_list *map_limits = new point_list;
	map_limits->add_node(new point_node(-1.0,-4.0));
	map_limits->add_node(new point_node(-1.0,1.0));
	map_limits->add_node(new point_node(4.0,1.0));
	map_limits->add_node(new point_node(4.0,-4.0));
    test_map.add_arena_points(map_limits);

	// Add obstacles
    point_list *pol = new point_list;

	pol->add_node(new point_node(-0.5,0.0));
    pol->add_node(new point_node(1.0,0.0));
    pol->add_node(new point_node(0.5,-1.0));
    test_map.add_obstacle(new polygon(pol));
		
	pol = new point_list;
    pol->add_node(new point_node(3.0,-1.0));
    pol->add_node(new point_node(3.5,-2.5));
    pol->add_node(new point_node(2.5,-2.25));
    pol->add_node(new point_node(2.0,-0.0));
    test_map.add_obstacle(new polygon(pol));

	pol = new point_list;
    pol->add_node(new point_node(-0.5,-3.5));
    pol->add_node(new point_node(1.0,-3.5));
    pol->add_node(new point_node(1.5,-2.25));
    pol->add_node(new point_node(0.0,-2.0));
    pol->add_node(new point_node(-0.9,-2.25));
    test_map.add_obstacle(new polygon(pol));

    pol = new point_list;
    pol->add_node(new point_node(2,-1.2));
    pol->add_node(new point_node(0.5,-1.2));
    pol->add_node(new point_node(0.5,-1.8));
    pol->add_node(new point_node(2,-1.8));
    test_map.add_obstacle(new polygon(pol));

    cout << "Number of obstacles: " << test_map.obstacles -> size << endl;

	// Add gates
    pol = new point_list;
    pol->add_node(new point_node(2,-4));
    pol->add_node(new point_node(3,-4));
    pol->add_node(new point_node(3,-3.5));
    pol->add_node(new point_node(2,-3.5));
    test_map.add_gate(new polygon(pol));	

    pol = new point_list;
    pol->add_node(new point_node(-1,-1));
    pol->add_node(new point_node(-1,0));
    pol->add_node(new point_node(-0.5,0));
    pol->add_node(new point_node(-0.5,-1));
    test_map.add_gate(new polygon(pol));	
    
	test_map.merge_obstacles();

   	test_map.make_free_space_cells();

	test_map.set_robot_position(1, -2);
	// test_map.reduce_arena();

	Mat img_arena = test_map.plot_arena(800, 800, true);
	
    imshow("Arena", img_arena);
	waitKey(0);
	
	// test_map.print_info();
}

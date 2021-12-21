#include "Roadmap/roadmap.h"

using namespace std;

void test();

int main(){
	test();
	return 0;
};

void test()
{
	points_map test_map;
	
	point_list *map_limits = new point_list;
	map_limits->add_node(new point_node(-1.0,-4.0));
	map_limits->add_node(new point_node(-1.0,1.0));
	map_limits->add_node(new point_node(4.0,1.0));
	map_limits->add_node(new point_node(4.0,-4.0));
    test_map.add_arena_points(map_limits);
    
    point_list *pol = new point_list;
    pol->add_node(new point_node(0.0,0.0));
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
    pol->add_node(new point_node(2,-4));
    pol->add_node(new point_node(3,-4));
    pol->add_node(new point_node(3,-3.5));
    pol->add_node(new point_node(2,-3.5));
    test_map.add_gate(new polygon(pol));	
    Mat img_arena = test_map.plot_arena();
    
    imshow("Arena", img_arena);
	waitKey(0);
	
	test_map.set_robot_position(11.3, 22.4);
	test_map.print_info();
}

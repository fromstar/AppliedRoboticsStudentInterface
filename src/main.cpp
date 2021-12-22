#include "Roadmap/roadmap.h"
#include "clipper/cpp/clipper.hpp"
//#include "../../simulator/src/9_project_interface/include/utils.hpp"

using namespace std;

void test();

int main(){
    /*points_map map;
    point_list pl;
    double coordinate_obstacle [4][2] = {{0., 0.},  {0., 1.},  {1., 1.},  {1., 0.}};
    for (int i=0;i<4;i++)
    {
        pl.add_node(new point_node(coordinate_obstacle[i][0],coordinate_obstacle[i][1]));
    }
    map.add_obstacle(new polygon(&pl));
    
    double coordinate_obstacle_1 [4][2] = {{2., 4.},  {10., 3.},  {4., 2.},  {7., 4.}};
    for (int i=0;i<4;i++)
    {
        pl.add_node(new point_node(coordinate_obstacle_1[i][0],coordinate_obstacle_1[i][1]));
    }
    map.add_obstacle(new polygon(&pl));
    Mat img_arena = map.plot_arena();
    imshow("Arena", img_arena);
	waitKey(0);*/
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
    test_map.add_obstacle(new polygon(pol,2));
    
    pol = new point_list;
    pol->add_node(new point_node(3.0,-1.0));
    pol->add_node(new point_node(3.5,-2.5));
    pol->add_node(new point_node(2.5,-2.25));
    pol->add_node(new point_node(2.0,-0.0));
    test_map.add_obstacle(new polygon(pol,2));
    
    pol = new point_list;
    pol->add_node(new point_node(-0.5,-3.5));
    pol->add_node(new point_node(1.0,-3.5));
    pol->add_node(new point_node(1.5,-2.25));
    pol->add_node(new point_node(0.0,-2.0));
    pol->add_node(new point_node(-0.9,-2.25));
    test_map.add_obstacle(new polygon(pol,2));
    
    pol = new point_list;
    pol->add_node(new point_node(2,-4));
    pol->add_node(new point_node(3,-4));
    pol->add_node(new point_node(3,-3.5));
    pol->add_node(new point_node(2,-3.5));
    test_map.add_gate(new polygon(pol,2));	

    pol = new point_list;
    pol->add_node(new point_node(-1,0));
    pol->add_node(new point_node(-1,-1));
    pol->add_node(new point_node(-0.5,-1));
    pol->add_node(new point_node(-0.5,0));
    test_map.add_gate(new polygon(pol,2));	
    Mat img_arena = test_map.plot_arena();
    
    imshow("Arena", img_arena);
	waitKey(0);
	
	test_map.set_robot_position(11.3, 22.4);
	test_map.print_info();
}

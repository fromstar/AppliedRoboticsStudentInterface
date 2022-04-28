#include "Roadmap/roadmap.h"
//#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include <time.h>

using namespace std;

void test();
void test2();

int main(){
	test();
	return 0;
};


void test()
{	
	logger* log_test = new logger;
	log_test -> set_log_path("test_log.txt");

	log_test -> add_event("Code started\n");
	points_map test_map(log_test);
	
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

	log_test -> add_event("Created Roadmap");
	
	Robot* c_1 = new Robot("Catcher_1", catcher); 
	test_map.add_robot(c_1);
	
	Robot* f_1 = new Robot("Fugitive_1", fugitive); 
	test_map.add_robot(f_1);

	test_map.set_robot_position(c_1->ID, 1, -2);
	test_map.set_robot_position(f_1->ID, -0.5, -0.5);
	
	World_representation abstract_arena = World_representation(
											test_map.free_space,
											test_map.gates,
											test_map.robot,
											log_test
										  );
	abstract_arena.info();
	abstract_arena.to_pddl("problem_catcher.pddl");
	abstract_arena.to_pddl("problem_fugitive.pddl", "fugitive_catcher",
						   "fugitive_catcher", false, true);
	
	// test_map.reduce_arena();
	/*
	Mat img_arena = test_map.plot_arena(800, 800, true);
	
    imshow("Arena", img_arena);
	imwrite("Arena.png", img_arena);
	waitKey(0);
	*/
	// test_map.print_info();
}

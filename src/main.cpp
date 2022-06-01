#include "Roadmap/roadmap.h"
//#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include <time.h>
#include <iostream>
#include "Dubins/dubins.h"

using namespace std;

void test();
void test2();

int main()
{
	// test();
	logger *log_test = new logger("test_log.txt");

	for(int i=0; i<5; i++){
		log_test->add_event("Hello");
	};
	log_test -> info();

	log_test->clear();
	log_test -> info();

	return 0;
};

void test()
{
	logger *log_test = new logger;
	log_test->set_log_path("test_log.txt");

	log_test->add_event("Code started");
	points_map test_map(log_test);

	// Define arena limits

	point_list *map_limits = new point_list;
	map_limits->add_node(new point_node(-1.0, -3.5));
	map_limits->add_node(new point_node(-1.0, 1.5));
	map_limits->add_node(new point_node(4.0, 1.5));
	map_limits->add_node(new point_node(4.0, -3.5));
	test_map.add_arena_points(map_limits);

	// Add obstacles
	point_list *pol = new point_list;

	pol->add_node(new point_node(0.9204, 0.6513));
	pol->add_node(new point_node(0.89895, 0.702));
 	pol->add_node(new point_node(0.9516, 0.73515));
	pol->add_node(new point_node(0.9906, 0.7059));
	pol->add_node(new point_node(0.99255, 0.6903));
	pol->add_node(new point_node(0.975, 0.65325));

	test_map.add_obstacle(new polygon(pol));

	pol = new point_list;
	pol->add_node(new point_node(0.7137, 0.64935));
	pol->add_node(new point_node(0.7098, 0.6591));
	pol->add_node(new point_node(0.77025, 0.75465));
	pol->add_node(new point_node(0.77805, 0.78585));
	pol->add_node(new point_node(0.7917, 0.78585));
	pol->add_node(new point_node(0.86775, 0.6552));
	test_map.add_obstacle(new polygon(pol));

	pol = new point_list;
	pol->add_node(new point_node(0.5109, 0.6006));
	pol->add_node(new point_node(0.5655, 0.70395));
	pol->add_node(new point_node(0.5772, 0.70395));
	pol->add_node(new point_node(0.62985, 0.60645));
	pol->add_node(new point_node(0.61035, 0.59865));
	test_map.add_obstacle(new polygon(pol));

	pol = new point_list;
	pol->add_node(new point_node(0.33345, 0.60255));
	pol->add_node(new point_node(0.3042, 0.68445));
	pol->add_node(new point_node(0.36855, 0.72735));
	pol->add_node(new point_node(0.39585, 0.7215));
	pol->add_node(new point_node(0.44265, 0.6864));
	pol->add_node(new point_node(0.44265, 0.663));
	pol->add_node(new point_node(0.4173, 0.6006));
	test_map.add_obstacle(new polygon(pol));

	pol = new point_list;
	pol->add_node(new point_node(0.25545, 0.6006));
	pol->add_node(new point_node(0.1794, 0.6006));
	pol->add_node(new point_node(0.21255, 0.6708));
	pol->add_node(new point_node(0.234, 0.6513));
	test_map.add_obstacle(new polygon(pol));

	pol = new point_list;
	pol->add_node(new point_node(0.05655, 0.5967));
	pol->add_node(new point_node(0.04095, 0.65715));
	pol->add_node(new point_node(0.0741, 0.6825));
	pol->add_node(new point_node(0.09945, 0.68445));
	pol->add_node(new point_node(0.13845, 0.65325));
	pol->add_node(new point_node(0.117, 0.59865));
	test_map.add_obstacle(new polygon(pol));

	pol = new point_list;
	pol->add_node(new point_node(0.94575, 0.45435));
	pol->add_node(new point_node(0.9009, 0.53625));
	pol->add_node(new point_node(0.94965, 0.62205));
	pol->add_node(new point_node(1.04325, 0.6201));
	pol->add_node(new point_node(1.09005, 0.5421));
	pol->add_node(new point_node(1.0725, 0.49725));
	pol->add_node(new point_node(1.04325, 0.4524));
	test_map.add_obstacle(new polygon(pol));

	pol = new point_list;
	pol->add_node(new point_node(0.7098, 0.45435));
	pol->add_node(new point_node(0.7098, 0.58695));
	pol->add_node(new point_node(0.8385, 0.59085));
	pol->add_node(new point_node(0.8346, 0.45435));
	test_map.add_obstacle(new polygon(pol));

	pol = new point_list;
	pol->add_node(new point_node(0.50505, 0.45825));
	pol->add_node(new point_node(0.50895, 0.55575));
	pol->add_node(new point_node(0.6084, 0.5538));
	pol->add_node(new point_node(0.60645, 0.4524));
	pol->add_node(new point_node(0.52455, 0.45045));
	test_map.add_obstacle(new polygon(pol));

	pol = new point_list;
	pol->add_node(new point_node(0.41535, 0.45435));
	pol->add_node(new point_node(0.35295, 0.45045));
	pol->add_node(new point_node(0.3315, 0.4602));
	pol->add_node(new point_node(0.30225, 0.5109));
	pol->add_node(new point_node(0.34125, 0.57525));
	pol->add_node(new point_node(0.4173, 0.5772));
	pol->add_node(new point_node(0.4524, 0.52065));
	test_map.add_obstacle(new polygon(pol));

	pol = new point_list;
	pol->add_node(new point_node(0.1755, 0.44655));
	pol->add_node(new point_node(0.1755, 0.51285));
	pol->add_node(new point_node(0.20475, 0.5226));
	pol->add_node(new point_node(0.23985, 0.5187));
	pol->add_node(new point_node(0.24765, 0.45045));
	test_map.add_obstacle(new polygon(pol));
	cout << "Number of obstacles: " << test_map.obstacles->size << endl;

	pol = new point_list;
	pol->add_node(new point_node(0.117, 0.4524));
	pol->add_node(new point_node(0.09165, 0.44655));
	pol->add_node(new point_node(0.06045, 0.4563));
	pol->add_node(new point_node(0.04095, 0.4953));
	pol->add_node(new point_node(0.0663, 0.5343));
	pol->add_node(new point_node(0.12285, 0.5265));
	pol->add_node(new point_node(0.13845, 0.48165));
	test_map.add_obstacle(new polygon(pol));

	// Add gates
	pol = new point_list;
	pol->add_node(new point_node(1.1973, 0.94965));
	pol->add_node(new point_node(1.1934, 1.04325));
	pol->add_node(new point_node(1.39815, 1.0452));
	pol->add_node(new point_node(1.4001, 0.9516));
	test_map.add_gate(new polygon(pol));

	/*
	polygon *gate_pol = new polygon(pol);
	Polygon_boost gate_ob = gate_pol->to_boost_polygon();

	polygon *p_arena = new polygon(map_limits);
	Polygon_boost boost_arena = p_arena->to_boost_polygon();

	if(boost::geometry::covered_by(gate_ob, boost_arena)){
		cout << "Gate inside arena" << endl;
	};
	*/

	/*
	pol = new point_list;
	pol->add_node(new point_node(-1, -1));
	pol->add_node(new point_node(-1, 0));
	pol->add_node(new point_node(-0.5, 0));
	pol->add_node(new point_node(-0.5, -1));
	test_map.add_gate(new polygon(pol));
	*/
	test_map.merge_obstacles();

	test_map.make_free_space_cells_squares(3);

	log_test->add_event("Created Roadmap");

	Robot *c_1 = new Robot("catcher_1", catcher);
	test_map.add_robot(c_1);

	Robot *f_1 = new Robot("fugitive_1", fugitive);
	test_map.add_robot(f_1);

	test_map.set_robot_position(c_1->ID, 1, -2, 0);
	test_map.set_robot_position(f_1->ID, -0.5, -0.5, 0);
	
	World_representation abstract_arena = World_representation(
		test_map.free_space,
		test_map.gates,
		log_test);
	abstract_arena.info();
	
	robot_manager rm;	

	rm.parse_map_robots(test_map.robot);
	rm.trade_fugitives();

	map<string, robot_fugitive*>::iterator it;

	it = rm.fugitives.begin();
	it->second->set_behaviour(aware);
	it -> second -> make_pddl_domain_file(abstract_arena);
	it -> second -> make_pddl_problem_file(abstract_arena);

	// it = rm.catchers.begin();
	// it->second -> make_pddl_files(abstract_arena);
	// rm.info(true);

	/*
	abstract_arena.to_pddl("Pddl/problem_catcher.pddl");
	abstract_arena.to_pddl("Pddl/problem_fugitive.pddl", "fugitive_catcher",
						   "fugitive_catcher", true);
	*/

	// test_map.reduce_arena();

	Mat img_arena = test_map.plot_arena(800, 800, true);

	/*Draw dubins path*/
	// vector<string> f_path = abstract_arena.world_robots["Fugitive_1"]->plan;

	// cout << abstract_arena.world_free_cells["Cell_1"].cell->centroid->y<<endl;
	
	/*
	double fx_path[f_path.size()+1];
	double fy_path[f_path.size()+1];
	double fth_path[f_path.size()+1];

	for (int i = 0; i < f_path.size(); i++)
	{
		string word;
		stringstream iss(f_path[i]);
		vector<string> path;
		while (iss >> word)
			path.push_back(word);
		path[3].resize(path[3].size() - 1);

		fx_path[i] = abstract_arena.world_free_cells[path[2]].cell->centroid->x;
		fy_path[i] = abstract_arena.world_free_cells[path[2]].cell->centroid->y;
		fth_path[i] = 0;

		if (i == f_path.size() - 1)
		{
			fx_path[i+1] = abstract_arena.world_gates[path[3]].cell->centroid->x;
			fy_path[i+1] = abstract_arena.world_gates[path[3]].cell->centroid->y;
			fth_path[i+1] = fth_path[i+1] + 10;
		}
	}

	int pidx;
	curve c;	
	for(int i=0;i<f_path.size();i++)
	{
		tie(pidx,c) = dubins(fx_path[i], fy_path[i], fth_path[i], fx_path[i+1],
							 fy_path[i+1], fth_path[i+1], 10);
		if(pidx > 0)
			img_arena = plotdubins(c, "r","g","b", img_arena);
	}
	*/

	/*End draw dubins path*/

	imshow("Arena", img_arena);
	imwrite("Arena.png", img_arena);
	waitKey(0);

	// test_map.print_info();
}

#include "robot_manager.h"

/**
 * \fun
 * This is the default constructor of the robot_manager struct.
 * @param l: logger\*. It is the logger instance which will be used to save
 * the events.
 */
robot_manager::robot_manager(logger *l)
{
	rm_logger = l;
};

/**
 * \fun
 * This method allows to add a robot to the struct.
 * @param r: Robot\*. It is a pointer to the Robot instance to be added to
 * the manager.
 * @param f_path: string. It is the path in which the robot will save files
 * necessary to perform its tasks.
 */
void robot_manager::add_robot(Robot *r, string f_path)
{
	switch (r->type)
	{
	case catcher:
		catchers[r->ID] = new robot_catcher(r, f_path);
		break;
	case fugitive:
		fugitives[r->ID] = new robot_fugitive(r, f_path);
		break;
	default:
		// All robots non fugitives are treated as potential threats for
		// the others hence catchers.
		catchers[r->ID] = new robot_catcher(r, f_path);
		break;
	};
	rm_logger->add_event("Added robot " + r->ID + " to the robot manager");
	if (trading_done)
	{
		rm_logger->add_event("Robot added when trading already done."
							 " Trading runs again.");
		trade_fugitives();
	};
};

/**
 * \fun
 * This method allows to parse a map of Robot instance and add its content
 * to the menager.
 * @param map_r: map<string, Robot\*>. It is the map containing the Robot
 * instances using as keys the ID of the robots.
 * @param f_path: string. It is the path in which the added robots will save
 * the files necessary for their execution.
 */
void robot_manager::parse_map_robots(map<string, Robot *> map_r, string f_path)
{
	for (map<string, Robot *>::iterator it = map_r.begin(); it != map_r.end();
		 ++it)
	{
		add_robot(it->second, f_path);
	};
	rm_logger->add_event("After parsing added " + map_r.size() +
						 string(" robots"));
};

/**
 * \fun
 * This method is used to chose the antagonist of each robot.
 * It works under geometrical assumptions.
 */
void robot_manager::trade_fugitives()
{
	if (fugitives.size() == catchers.size())
	{
		// One robot is the antagonist of the other
		map<string, robot_fugitive *>::iterator it_f = fugitives.begin();
		map<string, robot_catcher *>::iterator it_c = catchers.begin();

		it_f->second->add_antagonist(it_c->second->self);
		it_c->second->add_antagonist(it_f->second->self);
	}
	else
	{

		// Add other options

		if (fugitives.size() > catchers.size())
		{
			string warning_msg = "Fugitives are more than the catchers, "
								 "a solution may not be found.";
			cout << warning_msg << endl;
			rm_logger->add_event(warning_msg);
		}
		else
		{
			rm_logger->add_event("Catchers are more than the fugitives.");
			// How do we stop all the catchers when the fugitives have been
			// captured? i.e. how to broadcast the capture of a fugitive
			// to all the catchers?
		};

		/*
		 * In both cases the antagonists of ech type of robot are the ones
		 * nearest at them. This because, when a catchers wants to maximize the
		 * probability to win the task in a cooperative environment it will
		 * try to reach the nearest fugitives to capture them.
		 * In the same way, a fugitive which desire is to escape will see
		 * as the most iportant threat the nearest catchers.
		 * So, it is reasonable to employ the euclidean distance as a
		 * raw heuristic in chosing how to assign the antagonists.
		 */
		int num_antagonists_c = int(fugitives.size() / catchers.size());
		int num_antagonists_f = int(catchers.size() / fugitives.size());

		map<double, Robot *> distance;
		map<double, Robot *>::iterator it_low;

		// Assign antagonists to catchers
		map<string, robot_catcher *>::iterator it_c;
		map<string, robot_fugitive *>::iterator it_f;

		for (it_c = catchers.begin(); it_c != catchers.end(); ++it_c)
		{
			for (it_f = fugitives.begin(); it_f != fugitives.end(); ++it_f)
			{
				double x_diff = it_f->second->self->location->x -
								it_c->second->self->location->x;
				double y_diff = it_f->second->self->location->y -
								it_c->second->self->location->y;
				double dis = sqrt((pow(x_diff, 2) + pow(y_diff, 2)));
				distance[dis] = it_f->second->self;
			};
			int i = 0;
			do
			{
				it_low = distance.lower_bound(0); // Retrieve nearest

				// Add nearest as antagonist of the catcher
				it_c->second->add_antagonist(it_low->second);

				// Add the catcher as mutual antagonist of the fugitive
				it_f = fugitives.find(it_low->second->ID);
				it_f->second->add_antagonist(it_c->second->self);

				// Remove nearest from the map
				distance.erase(it_low);

				i++;
			} while (i < num_antagonists_c);
		};
	};
	// end options

	trading_done = true;
	rm_logger->add_event("Ended Robot trading");
};

/**
 * \fun
 * This method is used to print relevant informations regarding the robot
 * manager.
 * @param detailed: bool. It is a flag telling whether or not to show also
 * the informations regarding the robots contained in the robot manager.
 * It is setted to false by default.
 */
void robot_manager::info(bool detailed)
{
	cout << "Robot Menager info:" << endl
		 << "\tCatchers available: " << catchers.size() << endl
		 << "\tFugitives available: " << fugitives.size() << endl;
	if (detailed)
	{
		map<string, robot_fugitive *>::iterator it_f;
		map<string, robot_catcher *>::iterator it_c;

		cout << string(80, '-') << endl;
		// print info for fugitives
		for (it_f = fugitives.begin(); it_f != fugitives.end(); ++it_f)
		{
			it_f->second->info();
			cout << endl;
		};
		// print info for catchers
		for (it_c = catchers.begin(); it_c != catchers.end(); ++it_c)
		{
			it_c->second->info();
			cout << endl;
		};
		cout << string(80, '-') << endl;
	};
};


void robot_manager::run_agents_planners(World_representation wr,
										behaviour_fugitive supposed_fugitive_behaviour)
{
	rm_logger->add_event("Started threads for the fugitives");
	// Run fugitives threads
	map<string, robot_fugitive *>::iterator f_it;

	for (f_it = fugitives.begin(); f_it != fugitives.end(); f_it++)
	{
		f_it->second->make_pddl_domain_file();
		f_it->second->make_pddl_problem_file(wr);
	};

	rm_logger->add_event("Started threads for the catchers");
	// Run catchers threads
	map<string, robot_catcher *>::iterator c_it;

	for (c_it = catchers.begin(); c_it != catchers.end(); c_it++)
	{
		c_it->second->make_pddl_files(wr, supposed_fugitive_behaviour);
	};
};

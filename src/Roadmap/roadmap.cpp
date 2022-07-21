#include "roadmap.h"
#include <vector>
#include <sys/stat.h> // used to create folders
#include <iostream>
#include <fstream>
#include "../Connector/connector.h"

// #include <boost/geometry.hpp>
// #include <boost/geometry/geometries/point_xy.hpp>
// #include <boost/geometry/geometries/polygon.hpp>
// #include <boost/geometry/io/wkt/wkt.hpp>

// #include <boost/foreach.hpp>
// #include <boost/polygon/voronoi_builder.hpp>
// #include <boost/polygon/voronoi_diagram.hpp>
// #include <string.h>

namespace bg = boost::geometry;
namespace bgm = bg::model;

using pt = bgm::d2::point_xy<double>;
using Edge_boost = bgm::segment<pt>;
using Polygon_boost = bgm::polygon<pt>;
using Multi_Polygon_boost = bgm::multi_polygon<Polygon_boost>;

points_map::points_map(logger *l)
{
	log = l;
	connections = Connection_map(l);
}

void points_map::add_arena_points(point_list *ArenaPoints, double offset)
{
	arena = ArenaPoints;
	shrink_arena(offset);
};

void points_map::shrink_arena(double offset)
{
	if (shrinked_arena != NULL)
		return;

	polygon *p = new polygon(arena);

	shrinked_arena = new point_list();
	point_node *p1 = arena->head;

	double new_x, new_y;

	while (p1 != NULL)
	{
		if (p1->x < p->centroid->x)
		{
			new_x = p1->x + offset;
		}
		else
		{
			new_x = p1->x - offset;
		}

		if (p1->y < p->centroid->y)
		{
			new_y = p1->y + offset;
		}
		else
		{
			new_y = p1->y - offset;
		}

		shrinked_arena->add_node(new point_node(new_x, new_y));

		p1 = p1->pnext;
	}
}

/**
 * \fun void points_map::add_robot(Robot\* r)
 * Use this function to add a robot to the mapping of the environments.
 * The functions checks whether a robot with the same id exists in the mapping
 * and if so it changes the id adding an index at its end.
 * @param r: Robot. Is the Robot object to add to the mapping.
 */
void points_map::add_robot(Robot *r)
{
	int existing = robot.count(r->ID);
	if (existing > 0)
	{ // A robot with same id already exists.
		r->ID += "_" + to_string(existing);
		add_robot(r); // Recursive call, dangerous but ensures uniqueness.
	}
	else
	{ // No robot with that id
		robot[r->ID] = r;
	};
};

/**
 * \fun
 * This method allows to set the position for a robot in the structure given
 * its id.
 * @param robot_id: string. It is the id of the robot whic position has to be
 * set.
 * @param x: double. It is the x coordinate of the new position.
 * @param y: double. It is the y coordinate of the new position.
 * @param th: double. It is the orientation in radiants of the robot in the new
 * position.
 */
void points_map::set_robot_position(string robot_id, double x, double y, double th = 0)
{
	int existing = robot.count(robot_id);
	if (existing == 0)
	{
		cout << "No Robot found having id = \"" << robot_id << "\"" << endl;
		return;
	};
	Robot *_robot = robot[robot_id]; // If no element in container it adds it.
	delete (_robot->location);		 // free memory of previous location.
	_robot->location = new point_node(x, y);
	_robot->theta = th;
};

/**
 * \fun
 * This function is used to add a polygon representing a gate to the
 * pointsmap. It is a shortcut command.
 * @param gt: polygon\*. It is a pointer to a polygon instance representing a
 * gate in the environment.
 */
void points_map::add_gate(polygon *gt)
{
	gates->add_polygon(gt);
};

/**
 * \fun
 * This method allows to add an obstacle to the points map structure.
 * @param ob: polygon\*. It is a pointer to a polygon representing an obstacle
 * in the environment.
 */
void points_map::add_obstacle(polygon *ob)
{
	obstacles->size++;
	obstacles->offset_size++;
	polygon *offsetted_ob = ob->add_offset(obstacles->offset);
	offsetted_ob->id = "Offsetted_" + ob->id;
	if (obstacles->head == NULL)
	{
		obstacles->head = ob;
		obstacles->tail = obstacles->head;
		obstacles->offset_head = offsetted_ob;
		obstacles->offset_tail = obstacles->offset_head;
		return;
	}
	obstacles->tail->pnext = ob;
	obstacles->tail = obstacles->tail->pnext;
	obstacles->offset_tail->pnext = offsetted_ob;
	obstacles->offset_tail = obstacles->offset_tail->pnext;
};

/**
 * \fun
 * This method allows to print relevant informations regarding the
 * points map structure.
 */
void points_map::print_info()
{
	// Print robots informations.
	for (map<string, Robot *>::iterator robot_it = robot.begin();
		 robot_it != robot.end(); ++robot_it)
	{
		robot_it->second->info();
		cout << endl;
	};
};

/**
 * \fun
 * This method is used to return an image representing the environment.
 * @param x_dim: int. It is the width of the image.
 * @param y_dim: int. It is the height of the image.
 * @param show_original_polygons: bool. It is a flag telling whether or not
 * to show the polygons as they are together with their offsetted
 * representation (true) or not (false).
 * @param show_cells_id: bool. Flag to plot the cell id in the arena img.
 */
Mat points_map::plot_arena(int x_dim, int y_dim, bool show_original_polygons,
						   bool show_cells_id)
{
	Mat img_arena(x_dim, y_dim, CV_8UC3, Scalar(255, 255, 255));

	img_arena = plot_points(arena, img_arena, Scalar(0, 0, 0), true);
	// img_arena = plot_points(shrinked_arena, img_arena, Scalar(121, 18, 65), true);

	polygon *tmp = NULL;

	if (show_original_polygons)
	{
		// Plot obstacles as they are
		tmp = obstacles->head;
		while (tmp != NULL)
		{
			img_arena = plot_points(tmp->pl, img_arena, Scalar(255, 0, 0), true);
			tmp = tmp->pnext;
		}
	};

	// Plot obstacles enlarged
	tmp = obstacles->offset_head;
	while (tmp != NULL)
	{
		img_arena = plot_points(tmp->pl, img_arena, Scalar(0, 0, 255), true);
		tmp = tmp->pnext;
	}

	// plot gates
	tmp = gates->head;
	while (tmp != NULL)
	{
		img_arena = plot_points(tmp->pl, img_arena, Scalar(0, 255, 0), true);

		if (show_cells_id)
		{
			// Write ID of cell
			double x = ((tmp->centroid->x - 0.05) * SCALE_1) + SCALE_2;
			double y = (tmp->centroid->y * -SCALE_1) + SCALE_2;
			cv::putText(img_arena, tmp->id,
						cv::Point(x, y),
						cv::FONT_HERSHEY_DUPLEX, 0.3, Scalar(0, 0, 0));
		}

		tmp = tmp->pnext;
	}

	// plot free_space
	cout << endl
		 << "Plotting free space" << endl;
	tmp = free_space->head;
	while (tmp != NULL)
	{
		img_arena = plot_points(tmp->pl, img_arena, Scalar(0, 255, 255),
								true, 1);

		point_list *free_space_centroid = new point_list;
		free_space_centroid->add_node(tmp->centroid->copy());
		free_space_centroid->add_node(tmp->centroid->copy());
		img_arena = plot_points(free_space_centroid, img_arena,
								Scalar(0, 255, 255), false, 2);
		if (show_cells_id)
		{
			// Write ID of cell
			double x = ((tmp->centroid->x - 0.05) * SCALE_1) + SCALE_2;
			double y = (tmp->centroid->y * -SCALE_1) + SCALE_2;
			cv::putText(img_arena, tmp->id,
						cv::Point(x, y),
						cv::FONT_HERSHEY_DUPLEX, 0.3, Scalar(0, 0, 0));
		}
		tmp = tmp->pnext;
	};

	// plot robots

	for (map<string, Robot *>::iterator robot_it = robot.begin();
		 robot_it != robot.end(); ++robot_it)
	{

		point_list *robot_loc = new point_list;
		robot_loc->add_node(robot_it->second->location->copy());
		robot_loc->add_node(robot_it->second->location->copy());
		img_arena = plot_points(robot_loc, img_arena, Scalar(210, 26, 198),
								false, 5);
		delete (robot_loc);
	};

	return img_arena;
}

/**
 * This function is used to merge the obstacles that touch each others in the
 * arena space.
 */
void points_map::merge_obstacles()
{
	std::vector<Polygon_boost> polys;

	polygon *pol_iter = obstacles->offset_head;

	// Convert polygons in Boost polygon object
	while (pol_iter != NULL)
	{
		polys.push_back(pol_iter->to_boost_polygon());
		pol_iter = pol_iter->pnext;
	}

	// check which polygons intersect
	vector<Polygon_boost> output;
	int i = 0; // number of polygons present
	int psize = polys.size();
	while (i < psize)
	{
		int j = i + 1;
		bool lock = false;
		while (j < psize && lock == false)
		{
			if (boost::geometry::intersects(polys[i], polys[j]))
			{
				// Update the polygon list
				boost::geometry::union_(polys[i], polys[j], output);
				polys[i] = output[output.size() - 1];
				polys.erase(polys.begin() + j);
				psize--;
				j--;
			}
			j++;
		}
		i++;
	}

	// Check intersections with arena
	polygon *arena_in_use = NULL;

	if (arena != NULL)
	{
		arena_in_use = new polygon(arena);
	}
	else
	{
		printf("No arena points setted. Unable to check intersections.");
	};

	if (arena_in_use != NULL)
	{
		Polygon_boost _arena = arena_in_use->to_boost_polygon();
		vector<Polygon_boost> tmp_pols;

		for (int k = 0; k < i; k++)
		{
			if (boost::geometry::intersects(_arena, polys[k]))
			{
				boost::geometry::intersection(polys[k], _arena, tmp_pols);
				polys[k] = tmp_pols[tmp_pols.size() - 1];
			};
		};
	}

	// Check intersections with gates
	if (gates != NULL)
	{
		polygon *tmp_gate = gates->head;
		gates = NULL; // Delete old gates list
		vector<Polygon_boost> tmp_new_gates;
		points_map *tmp_map = new points_map(NULL);

		while (tmp_gate != NULL)
		{
			Polygon_boost _gate = tmp_gate->to_boost_polygon();
			for (int k = 0; k < i; k++)
			{
				if (boost::geometry::intersects(_gate, polys[k]))
				{
					boost::geometry::difference(_gate, polys[k],
												tmp_new_gates);
					point_list *pl = boost_polygon_to_point_list(
						tmp_new_gates[tmp_new_gates.size() - 1]);
					polygon *new_gate = new polygon(pl);
					new_gate->pnext = tmp_gate->pnext;
					tmp_gate = new_gate;
				};
			};
			tmp_map->add_gate(tmp_gate);
			tmp_gate = tmp_gate->pnext;
		};
		gates = tmp_map->gates;
	};

	// Delete the offsetted list
	obstacles->delete_offsetted_list();

	// Repopulate with updatate offsetted polygons
	for (i = 0; i < polys.size(); i++)
	{
		polygon *pl = boost_polygon_to_polygon(polys[i]);

		if (obstacles->offset_head == NULL)
		{
			obstacles->offset_head = pl;
			obstacles->offset_tail = obstacles->offset_head;
		}
		else
		{
			obstacles->offset_tail->pnext = pl;
			obstacles->offset_tail = obstacles->offset_tail->pnext;
		}
		obstacles->offset_size++;
	}

	// If gate is inside obstacles remove the gate
	polygon *tmp_gate = gates->head;
	polygon *tmp_gate_prev = NULL;
	while (tmp_gate != NULL)
	{
		bool interrupt = false;
		Polygon_boost gate_boost = tmp_gate->to_boost_polygon();
		polygon *tmp_ob = obstacles->offset_head;
		while (tmp_ob != NULL && interrupt == false)
		{
			Polygon_boost ob_boost = tmp_ob->to_boost_polygon();

			if (bg::within(gate_boost, ob_boost))
			{
				// Remove gate from the list
				// Case gate is at the start of the list
				if (tmp_gate == gates->head)
				{
					gates->head = tmp_gate->pnext;
					delete tmp_gate;
				}
				else
				{
					tmp_gate_prev->pnext = tmp_gate->pnext;
					delete tmp_gate;
					tmp_gate = tmp_gate_prev;
					if (tmp_gate == gates->tail)
					{
						gates->tail = tmp_gate_prev;
					}
				};
				interrupt = true; // Avoid parsing further obstacles
			};

			tmp_ob = tmp_ob->pnext;
		};
		tmp_gate_prev = tmp_gate;
		tmp_gate = tmp_gate->pnext;
	};

	log->add_event("Obstacles in touch merged");
}

/**
 * \fun
 * This function is used to make the convex hull of an obstacle in
 * the points map.
 */
void points_map::convexify_obstacles()
{
	polygon *pol = obstacles->offset_head;
	while (pol != NULL)
	{
		// convert to boost
		Polygon_boost boost_pol = pol->to_boost_polygon();
		Polygon_boost output;
		bg::convex_hull(boost_pol, output);
		pol->pl->delete_list(); // Remove old allocation of data
		pol->pl = boost_polygon_to_polygon(output)->pl;
		pol = pol->pnext;
	};

	// If gate is inside obstacle remove the gate
	polygon *tmp_gate = gates->head;
	polygon *tmp_gate_prev = NULL;
	while (tmp_gate != NULL)
	{
		bool interrupt = false;
		Polygon_boost gate_boost = tmp_gate->to_boost_polygon();
		polygon *tmp_ob = obstacles->offset_head;
		while (tmp_ob != NULL && interrupt == false)
		{
			Polygon_boost ob_boost = tmp_ob->to_boost_polygon();

			if (bg::within(gate_boost, ob_boost))
			{
				// Remove gate from the list
				// Case gate is at the start of the list
				if (tmp_gate == gates->head)
				{
					gates->head = tmp_gate->pnext;
					delete tmp_gate;
				}
				else
				{
					tmp_gate_prev->pnext = tmp_gate->pnext;
					delete tmp_gate;
					tmp_gate = tmp_gate_prev;
					if (tmp_gate == gates->tail)
					{
						gates->tail = tmp_gate_prev;
					}
				};
				interrupt = true; // Avoid parsing further obstacles
			};

			tmp_ob = tmp_ob->pnext;
		};
		tmp_gate_prev = tmp_gate;
		tmp_gate = tmp_gate->pnext;
	};

	if (gates != NULL)
	{

		std::vector<Polygon_boost> polys;
		polygon *pol_iter = obstacles->offset_head;
		// Convert polygons in Boost polygon object
		while (pol_iter != NULL)
		{
			polys.push_back(pol_iter->to_boost_polygon());
			pol_iter = pol_iter->pnext;
		}

		polygon *tmp_gate = gates->head;
		gates = NULL; // Delete old gates list
		vector<Polygon_boost> tmp_new_gates;
		points_map *tmp_map = new points_map(NULL);

		while (tmp_gate != NULL)
		{
			Polygon_boost _gate = tmp_gate->to_boost_polygon();
			for (int k = 0; k < polys.size(); k++)
			{
				if (boost::geometry::intersects(_gate, polys[k]))
				{
					boost::geometry::difference(_gate, polys[k],
												tmp_new_gates);
					point_list *pl = boost_polygon_to_point_list(
						tmp_new_gates[tmp_new_gates.size() - 1]);
					polygon *new_gate = new polygon(pl);
					new_gate->pnext = tmp_gate->pnext;
					tmp_gate = new_gate;
				};
			};
			tmp_map->add_gate(tmp_gate);
			tmp_gate = tmp_gate->pnext;
		};
		gates = tmp_map->gates;
	};
};

/**
 * This function is used to subset a polygon into a finite number of cells.
 * The cells are formed creating a triange connecting every edge of the polygon
 * with the centroid of the object. The number of cells is num_edges^(1+levels).
 * @param[in] levels: int. Is the number of iterations to perform during the
 * polygon subsetting.
 * @param[out] subset_list: list_of_polygons.
 */
list_of_polygons *subset_polygon(polygon *p, int levels)
{
	list_of_polygons *subset_list = new list_of_polygons;

	point_node *tmp_point_start = p->pl->head;

	while (tmp_point_start->pnext != NULL)
	{
		point_list *tmp_point_list = new point_list;

		double s_x = tmp_point_start->x;
		double s_y = tmp_point_start->y;
		double e_x = tmp_point_start->pnext->x;
		double e_y = tmp_point_start->pnext->y;

		tmp_point_list->add_node(new point_node(s_x, s_y));
		tmp_point_list->add_node(new point_node(e_x, e_y));
		tmp_point_list->add_node(p->centroid);

		subset_list->add_polygon(new polygon(tmp_point_list));
		tmp_point_start = tmp_point_start->pnext;

		if (tmp_point_start->pnext == NULL)
		{
			tmp_point_list = new point_list;

			double s_x = tmp_point_start->x;
			double s_y = tmp_point_start->y;
			double e_x = p->pl->head->x;
			double e_y = p->pl->head->y;

			tmp_point_list->add_node(new point_node(s_x, s_y));
			tmp_point_list->add_node(new point_node(e_x, e_y));
			tmp_point_list->add_node(p->centroid);
			subset_list->add_polygon(new polygon(tmp_point_list));
		};
	};
	for (int i = 0; i < levels - 1; i++)
	{
		polygon *tmp_pol = subset_list->head;
		list_of_polygons *new_subset_list = new list_of_polygons;
		while (tmp_pol != NULL)
		{
			polygon *tmp_list = new polygon(tmp_pol->pl);
			new_subset_list->append_other_list(subset_polygon(tmp_list));
			tmp_pol = tmp_pol->pnext;
		};
		subset_list = new_subset_list;
	};
	return subset_list;
};

/**
 * Subtract a vector of polygons from another one.
 * @param arena: vector<Polygon_boost>. Is the vector of polygons from which
 * the other polygons will be subtracted.
 * @param obstacles: vector<Polygon_boost>. Is the vector of polygons that will
 * be subtracted.
 */

/*
vector<Polygon_boost> difference_of_vectors(vector<Polygon_boost> arena,
  vector<Polygon_boost> obstacles)
{

	if (arena.size() == 0)
	{
		cout << "Arena vector is empty." << endl;
	};
	if (obstacles.size() == 0)
	{
		cout << "Obstacle vector is empty." << endl;
	};

	vector<Polygon_boost> output;
	vector<Polygon_boost> tmp_output;

	int prev_output = 0;
	int arena_size = arena.size();
	int arena_ob_size = obstacles.size();

	for (int i = 0; i < arena_size; i++) // i is the index of the cell
	{
		int j = 0; // Index of the obstacle
		bool interrupt_ob = false;
		while (j < arena_ob_size && interrupt_ob == false)
		{
			// Apply only if cells are outside the obstacles
			if (bg::covered_by(arena[i], obstacles[j]) == false)
			{
				if (bg::intersects(arena[i], obstacles[j]))
				{
					// cout << "Intersects" << endl;
					bg::difference(arena[i], obstacles[j], output);
					int diff = output.size() - prev_output;
					prev_output = output.size();

					// cout << "Output size: " << output.size() << endl;
					// cout << "Difference: " << diff << " Size output: "
					// 	 << prev_output << endl;

					arena[i] = output[output.size() - 1];

					if (diff > 1)
					{
						cout << "In diff > 1" << endl;
						// tmp_output = arena;
						for (int k = 1; k < diff; k++)
						{
							int output_idx = output.size() - 1 - k;
							vector<Polygon_boost>::iterator it;
							// it = tmp_output.begin();
							it = arena.begin();
							// tmp_output
							// arena.insert(it + i + k, output[output_idx]);
							arena.push_back(output[output_idx]);
							arena_size++;
						};
						// arena = tmp_output;
					};

				};
			}
			else
			{
				// Delete cells covered by obstacles;
				// cout << "Covered" << endl;
				arena.erase(arena.begin() + i);
				i--;
				arena_size--;
				interrupt_ob = true;
			};
			j++;
		};
	};
	return arena;
};
*/

/*
list_of_polygons * subset_over_middle_point(polygon * p)
{
	list_of_polygons *tmp_output = new list_of_polygons();

	Edge_list *e_list = p->edgify();
	Edge *tmp = e_list->head;

	Edge *start = e_list->head;
	while (start != NULL)
	{
		Edge *end;
		if (start->next == NULL)
		{
			end = e_list->head;
		}
		else
		{
			end = start->next;
		};

		point_list *pl_temp = new point_list();
		pl_temp->add_node(new point_node(p->centroid->x, p->centroid->y));
		pl_temp->add_node(start->middle_point());
		pl_temp->add_node(start->points->tail);
		pl_temp->add_node(end->middle_point());

		polygon *cell = new polygon(pl_temp);
		tmp_output->add_polygon(cell);

		start = start->next;
	};
	return tmp_output;
}
*/

vector<Edge *> fine_tune_sweep_line(vector<Edge *> edges_ob,
									list_of_polygons *obstacles,
									polygon *limits)
{
	cout << "- * Refining sweep over " << edges_ob.size() << " edges" << endl;

	// For each sweep line edge
	polygon *pol = NULL;

	vector<Edge *> new_edges = edges_ob;

	int edges_size = edges_ob.size();

	pol = obstacles->head;
	while (pol != NULL)
	{
		Polygon_boost pol_boost = pol->to_boost_polygon();
		for (int i = 0; i < edges_size; i++)
		{
			cout << "Working on Polygon: " << pol->id << " for Edge " << i << endl;

			point_list *intersection_points = NULL;
			intersection_points = pol->intersections_with_edge(edges_ob[i]);

			cout << "Intersection points size: " << intersection_points->size << endl;

			// For all points in intersection with the polygon
			if (intersection_points != NULL && intersection_points->size > 0)
			{
				if (intersection_points->size == 1) // Only one intersection
				{
					cout << "Intersection points list has size == 1" << endl;
					point_node *inter = intersection_points->head;

					if (pol->is_vertex(inter) == true)
					{
						point_node *vertex = inter->copy();

						pt edge_point = edges_ob[i]->points->head->to_boost();

						Edge *new_edge_1 = NULL;
						Edge *new_edge_2 = NULL;
						if (!bg::within(edge_point, pol_boost))
						{
							new_edge_1 = new Edge(vertex, edges_ob[i]->points->head);
						}

						edge_point = edges_ob[i]->points->tail->to_boost();
						if (!bg::within(edge_point, pol_boost))
						{
							new_edge_2 = new Edge(vertex, edges_ob[i]->points->tail);
						}

						if (new_edge_1 != NULL && new_edge_2 != NULL)
						{
							edges_ob[i] = new_edge_1;
							edges_ob.push_back(new_edge_2);
						}
						else if (new_edge_1 != NULL)
						{
							edges_ob[i] = new_edge_1;
						}
						else if (new_edge_2 != NULL)
						{
							edges_ob[i] = new_edge_2;
						}
					}
					else // Line created by another polygon vertex -> find it
					{
						cout << "In this case" << endl;

						polygon *pol_iter = obstacles->head;
						point_node *found_point = NULL;

						point_node *head = edges_ob[i]->points->head;
						point_node *tail = edges_ob[i]->points->tail;
						int counter = 0;
						while (pol_iter != NULL && found_point == NULL)
						{
							found_point = edge_has_vertex(edges_ob[i],
														  pol_iter);

							pol_iter = pol_iter->pnext;
						}

						if (found_point != NULL)
						{
							edges_ob[i] = new Edge(found_point, inter);
							point_node *vertex = found_point;
							// Preserve other chunk
							if (head->y > tail->y)
							{
								if (inter->y > vertex->y &&
									vertex->y > tail->y &&
									vertex->y < head->y)
								{
									edges_ob.push_back(new Edge(vertex, tail));
									edges_size++;
								}
								else if (inter->y < vertex->y &&
										 vertex->y < head->y &&
										 vertex->y > tail->y)
								{
									edges_ob.push_back(new Edge(vertex, head));
									edges_size++;
								}
							}
							else if (head->y < tail->y)
							{
								if (inter->y > vertex->y &&
									vertex->y > head->y &&
									vertex->y < tail->y)
								{
									edges_ob.push_back(new Edge(vertex, head));
									edges_size++;
								}
								else if (inter->y < vertex->y &&
										 vertex->y > head->y &&
										 vertex->y < tail->y)
								{
									edges_ob.push_back(new Edge(vertex, tail));
									edges_size++;
								}
							}
						}
						else
						{
							cout << "Found point is NULL" << endl;
						}
					}
				}
				else if (intersection_points->size == 2)
				{
					cout << "Intersection points list size == 2" << endl;

					point_node *int_pt1 = intersection_points->head; // head
					point_node *int_pt2 = intersection_points->tail; // tail

					point_node *e_head = edges_ob[i]->points->head;
					point_node *e_tail = edges_ob[i]->points->tail;

					Edge *new_e = NULL;

					if (int_pt2->y < int_pt1->y)
					{
						if (pol->is_vertex(int_pt1) && pol->is_vertex(int_pt2))
						{
							new_e = new Edge(int_pt1, e_head);
							Edge *e2 = new Edge(int_pt2, e_tail);
							edges_ob[i] = new_e;
							edges_ob.push_back(e2);
						}
						else if (pol->is_vertex(int_pt1))
						{
							new_e = new Edge(int_pt1, e_head);
							edges_ob[i] = new_e;
						}
						else if (pol->is_vertex(int_pt2))
						{
							new_e = new Edge(int_pt2, e_tail);
							edges_ob[i] = new_e;
						}
						else // None of them is a vertex
						{
							Edge_list *e_l = new Edge_list();
							polygon *pol_iter = obstacles->head;
							bool head_vertex = false;
							bool tail_vertex = false;

							while (pol_iter != NULL &&
								   head_vertex == false &&
								   tail_vertex == false)
							{
								if (pol_iter->is_vertex(e_head))
								{
									head_vertex = true;
								}
								if (pol_iter->is_vertex(e_tail))
								{
									tail_vertex = true;
								}
								pol_iter = pol_iter->pnext;
							}

							if (head_vertex == true)
							{
								e_l->add_edge(new Edge(e_head, int_pt1));
							}
							if (tail_vertex == true)
							{
								e_l->add_edge(new Edge(e_tail, int_pt2));
							}

							if (!head_vertex && !tail_vertex)
							{
								// An original sweep_line -> no vertex at each
								// ends -> vertex is in between
								Edge *t_e_1 = new Edge(e_head, int_pt1);
								Edge *t_e_2 = new Edge(int_pt2, e_tail);

								bool t_e_1_ok = false;
								bool t_e_2_ok = false;

								// Validate new sub_edges
								polygon *new_pol_iter = obstacles->head;
								while (new_pol_iter != NULL &&
									   (!t_e_1_ok || !t_e_2_ok))
								{
									point_node *temp = NULL;

									if (!t_e_1_ok)
									{
										temp = edge_has_vertex(t_e_1,
															   new_pol_iter);
										if (temp != NULL)
										{
											t_e_1_ok = true;
										}
									}
									if (!t_e_2_ok)
									{
										temp = edge_has_vertex(t_e_2,
															   new_pol_iter);
										if (temp != NULL)
										{
											t_e_2_ok = true;
										}
									}
									new_pol_iter = new_pol_iter->pnext;
								}

								if (t_e_1_ok)
								{
									e_l->add_edge(t_e_1);
								}
								if (t_e_2_ok)
								{
									e_l->add_edge(t_e_2);
								}
							}

							new_e = e_l->head;
							bool updated = false;
							while (new_e != NULL)
							{
								if (updated == false)
								{
									edges_ob[i] = new_e;
									updated = true;
								}
								else
								{
									edges_ob.push_back(new_e);
								}
								new_e = new_e->next;
							}
						}
					}
					else
					{
						if (pol->is_vertex(int_pt1) && pol->is_vertex(int_pt2))
						{
							new_e = new Edge(int_pt1,
											 edges_ob[i]->points->tail);
							Edge *e2 = new Edge(int_pt2,
												edges_ob[i]->points->head);
							edges_ob[i] = new_e;
							edges_ob.push_back(e2);
						}
						else if (pol->is_vertex(int_pt1))
						{
							new_e = new Edge(int_pt1, edges_ob[i]->points->tail);
							edges_ob[i] = new_e;
						}
						else if (pol->is_vertex(int_pt2))
						{
							new_e = new Edge(int_pt2, edges_ob[i]->points->head);
							edges_ob[i] = new_e;
						}
						else // None of them is a vertex
						{
							Edge_list *e_l = new Edge_list();
							polygon *pol_iter = obstacles->head;
							bool head_vertex = false;
							bool tail_vertex = false;

							while (pol_iter != NULL &&
								   head_vertex == false &&
								   tail_vertex == false)
							{
								if (pol_iter->is_vertex(e_head))
								{
									head_vertex = true;
								}
								if (pol_iter->is_vertex(e_tail))
								{
									tail_vertex = true;
								}
								pol_iter = pol_iter->pnext;
							}

							if (head_vertex == true)
							{
								e_l->add_edge(new Edge(e_head, int_pt2));
							}
							if (tail_vertex == true)
							{
								e_l->add_edge(new Edge(e_tail, int_pt1));
							}

							if (!head_vertex && !tail_vertex)
							{
								// An original sweep_line -> no vertex at each
								// ends -> vertex is in between
								Edge *t_e_1 = new Edge(e_head, int_pt2);
								Edge *t_e_2 = new Edge(int_pt1, e_tail);

								bool t_e_1_ok = false;
								bool t_e_2_ok = false;

								// Validate new sub_edges
								polygon *new_pol_iter = obstacles->head;
								while (new_pol_iter != NULL &&
									   (!t_e_1_ok || !t_e_2_ok))
								{
									point_node *temp = NULL;
									if (!t_e_1_ok)
									{
										temp = edge_has_vertex(t_e_1,
															   new_pol_iter);
										if (temp != NULL)
										{
											t_e_1_ok = true;
										}
									}
									if (!t_e_2_ok)
									{
										temp = edge_has_vertex(t_e_2,
															   new_pol_iter);
										if (temp != NULL)
										{
											t_e_2_ok = true;
										}
									}
									new_pol_iter = new_pol_iter->pnext;
								}

								if (t_e_1_ok)
								{
									e_l->add_edge(t_e_1);
								}
								if (t_e_2_ok)
								{
									e_l->add_edge(t_e_2);
								}
							}

							new_e = e_l->head;
							bool updated = false;
							while (new_e != NULL)
							{
								if (updated == false)
								{
									edges_ob[i] = new_e;
									updated = true;
								}
								else
								{
									edges_ob.push_back(new_e);
								}
								new_e = new_e->next;
							}
						}
					}
				}
			}
		}
		edges_size = edges_ob.size();

		pol = pol->pnext;
	}

	// check if the limits are satisfied if any
	/*
	if (limits != NULL)
	{
		Edge_list * el = limits->edgify();
		Edge * edge_iter = el->head;

		while(edge_iter != NULL)
		{
			point_node * limit_head = edge_iter->points->head;
			point_node * limit_tail = edge_iter->points->tail;

			for(int i=0; i < edges_ob.size(); i++)
			{
				point_node * edge_head = edges_ob[i]->points->head;
				point_node * edge_tail = edges_ob[i]->points->tail;

				point_node * inter = edges_ob[i]->intersection(edge_iter);
				if (inter != NULL)
				{
				  if (inter != edge_head && inter != edge_tail)
				  {
					// One of them out -> reshape
					if(inter->y > edge_tail->y && edge_tail->y < edge_head->y)
					{
						edges_ob[i] = new Edge(edge_head, inter);
					}
					else if(inter->y < edge_tail->y && edge_tail->y > edge_head->y)
					{
						edges_ob[i] = new Edge(edge_head, inter);
					}
				  }
				}
			}

			edge_iter = edge_iter->next;
		}
	}
	*/
	cout << "Total edges: " << edges_ob.size() << endl;
	return edges_ob;
}

void points_map::make_exact_cell()
{
	convexify_obstacles();
	merge_obstacles();
	convexify_obstacles();

	double *low_y = NULL;
	double *high_y = NULL;
	double *low_x = NULL;
	double *high_x = NULL;

	point_node *tmp = shrinked_arena->head;

	// Find min and max y
	while (tmp != NULL)
	{
		if (low_y == NULL || tmp->y < *low_y)
		{
			low_y = &tmp->y;
		}

		if (high_y == NULL || tmp->y > *high_y)
		{
			high_y = &tmp->y;
		}

		if (low_x == NULL || tmp->x < *low_x)
		{
			low_x = &tmp->x;
		}

		if (high_x == NULL || tmp->x > *high_x)
		{
			high_x = &tmp->x;
		}
		tmp = tmp->pnext;
	}

	// Make sweep lines
	polygon *arena_pol = new polygon(shrinked_arena);
	list_of_polygons *polygons_to_parse = new list_of_polygons();
	polygon *pol = obstacles->offset_head;
	while (pol != NULL)
	{
		polygons_to_parse->add_polygon(pol);
		pol = pol->pnext;
	}
	polygons_to_parse->add_polygon(arena_pol);

	map<double, Edge *> sweep_lines;
	vector<Edge *> final_edges;
	Edge_list *arena_limits = arena_pol->edgify();

	Polygon_boost arena_boost = arena_pol->to_boost_polygon();
	boost::geometry::correct(arena_boost);

	pol = polygons_to_parse->head;
	while (pol != NULL)
	{
		// polygons_to_parse->add_polygon(pol->copy());
		point_node *temp_p = pol->pl->head; // polygon vertex

		pt vertex = temp_p->to_boost();

		Edge_list *pol_edge = pol->edgify();

		while (temp_p != NULL)
		{
			bool accepted = bg::covered_by(vertex, arena_boost);
			if (accepted)
			{
				point_node *start = new point_node(temp_p->x, *high_y);
				point_node *end = new point_node(temp_p->x, *low_y);

				Edge *v_sweep_edge = new Edge(start, end);
				if (sweep_lines.count(temp_p->x) == 0)
				{
					sweep_lines[temp_p->x] = v_sweep_edge;
					final_edges.push_back(v_sweep_edge);
				}
			}

			// edges_ob.push_back(v_sweep_edge);
			temp_p = temp_p->pnext;
			if (temp_p != NULL)
			{
				vertex = temp_p->to_boost();
			}
		}
		pol = pol->pnext;
	}

	int map_size = sweep_lines.size();
	vector<polygon *> cells;

	point_node *upper_left = NULL;
	point_node *lower_left = NULL;
	point_node *upper_right = NULL;
	point_node *lower_right = NULL;

	map<double, Edge *>::iterator it;
	/*
	for(it = sweep_lines.begin(); it != sweep_lines.end(); it++)
	{
		cout << it->first << " -> " << it->second->points->head->y <<
			 " : " << it->second->points->tail->y << endl;
	}
	*/

	for (int i = 0; i < map_size; i++)
	{
		it = sweep_lines.upper_bound(*low_x - 1e-6);
		// cout << it->first << " - " << sweep_lines.size() << endl;

		if (upper_left == NULL && lower_left == NULL) // first iteration
		{
			point_list *ord_list = it->second->points->orderify(1);
			upper_left = ord_list->tail;
			lower_left = ord_list->head;
		}
		else
		{
			point_list *ord_list = it->second->points->orderify(1);
			upper_right = ord_list->tail;
			lower_right = ord_list->head;

			point_list *pol_pl = new point_list();
			pol_pl->add_node(lower_left);
			pol_pl->add_node(upper_left);
			pol_pl->add_node(upper_right);
			pol_pl->add_node(lower_right);

			polygon *new_pol = new polygon(pol_pl, "CELL_" + to_string(i));
			cells.push_back(new_pol);

			upper_left = upper_right;
			lower_left = lower_right;
		}
		sweep_lines.erase(it);
	}

	// Reshape cells to arena limits
	vector<Polygon_boost> cells_boost;
	for (int i = 0; i < cells.size(); i++)
	{
		vector<Polygon_boost> output;

		Polygon_boost cell_boost = cells[i]->to_boost_polygon();
		cells_boost.push_back(cell_boost);

		if (bg::intersects(arena_boost, cell_boost))
		{
			bg::intersection(arena_boost, cell_boost, output);
			if (output.size() == 1)
			{
				polygon *p = boost_polygon_to_polygon(output[0], cells[i]->id);
				cells[i] = p;
			}
			else if (output.size() > 1)
			{
				cells[i] = boost_polygon_to_polygon(output[0], cells[i]->id);
				for (int j = 1; j < output.size(); j++)
				{
					string _id = cells[i]->id + "_" + to_string(j);
					cells.push_back(boost_polygon_to_polygon(output[j], _id));
				}
			}
		}
	}

	// Reshape cells to obstacles and populate connection map
	vector<Polygon_boost> ob_boost;
	pol = obstacles->offset_head;
	while (pol != NULL)
	{
		Polygon_boost ob_temp = pol->to_boost_polygon();
		ob_boost.push_back(ob_temp);
		pol = pol->pnext;
	}

	vector<Polygon_boost> reshaped_cells;
	reshaped_cells = difference_of_vectors(cells_boost, ob_boost);

	vector<polygon *> final_cells;
	for (int i = 0; i < reshaped_cells.size(); i++)
	{
		string _id = "CELL_" + to_string(i);
		polygon *temp_p = boost_polygon_to_polygon(reshaped_cells[i], _id);
		final_cells.push_back(temp_p);
		connections.add_element(temp_p);
	}

	// Merge cells
	for (int i = 0; i < final_cells.size(); i++)
	{
		polygon *p_now = final_cells[i];
		if (connections.connections.count(p_now->id) != 0)
		{
			Master_node pol_node = connections.connections[p_now->id];
			map<string, polygon *> conns = pol_node.adjacent_connections;

			map<string, polygon *>::const_iterator conn_it_2;
			for (conn_it_2 = conns.cbegin(); conn_it_2 != conns.cend();
				 conn_it_2++)
			{
				Edge *c_edge = find_common_edge(p_now,
												conn_it_2->second);

				if (c_edge != NULL)
				{
					point_node *vertex = NULL;

					pol = obstacles->offset_head;
					while (pol != NULL && vertex == NULL)
					{
						point_node *t = pol->pl->head;
						while (t != NULL && vertex == NULL)
						{
							if (*t == *c_edge->points->head ||
								*t == *c_edge->points->tail)
							{
								vertex = t->copy();
							}
							t = t->pnext;
						}
						pol = pol->pnext;
					}

					if (vertex == NULL)
					{
						connections.unify(p_now->id, conn_it_2->first);
						final_cells[i] = connections.connections[p_now->id].master;

						int index = -1;
						for (int j = 0; j < final_cells.size(); j++)
						{
							if (final_cells[j]->id == conn_it_2->first)
							{
								index = j;
							}
						}

						if (index != -1)
						{
							final_cells[index] = connections.connections[p_now->id].master;
						}
					}
				}
			}
		}
	}

	map<string, Master_node>::iterator dit;
	for (dit = connections.connections.begin(); dit != connections.connections.end(); dit++)
	{
		dit->second.diagonal_connections.clear();
	}
	map<string, polygon *> els = connections.elements();
	map<string, polygon *>::const_iterator c_el;
	for (c_el = els.cbegin(); c_el != els.cend(); c_el++)
	{
		free_space->add_polygon(c_el->second);
	}
}

/**
 * \fun
 * This method is used to create cells in the free space that
 * compose a free movement grid.
 * @param res: int. It is the resolution steps of the algorithm -> how many
 * 					times does it runs.
 */
void points_map::make_free_space_cells_squares(int res)
{
	// Convexify and merge obstacles
	convexify_obstacles();
	merge_obstacles();
	convexify_obstacles();

	// Generate arena subsetting
	list_of_polygons *tmp_list = new list_of_polygons();
	tmp_list->add_polygon(new polygon(shrinked_arena));

	polygon *tmp_pol = NULL;

	for (int i = 0; i < res; i++)
	{
		tmp_pol = tmp_list->head;
		list_of_polygons *tmp_output = new list_of_polygons();
		while (tmp_pol != NULL)
		{
			tmp_output->append_other_list(subset_over_middle_point(tmp_pol));
			tmp_pol = tmp_pol->pnext;
		};

		tmp_list = tmp_output;
	};

	tmp_pol = NULL;

	// convert cells to boost polygons
	vector<Polygon_boost> cells;
	tmp_pol = tmp_list->head;
	while (tmp_pol != NULL)
	{
		cells.push_back(tmp_pol->to_boost_polygon());
		tmp_pol = tmp_pol->pnext;
	};

	// convert obstacles to boost ones
	vector<Polygon_boost> ob_boost;
	tmp_pol = obstacles->offset_head;
	while (tmp_pol != NULL)
	{
		ob_boost.push_back(tmp_pol->to_boost_polygon());
		tmp_pol = tmp_pol->pnext;
	};

	cout << "Before difference of vectors" << endl;

	// Remove obstacles from the cells
	cells = difference_of_vectors(cells, ob_boost);

	cout << "After difference of vectors" << endl;

	// Populate free space cells list
	// list_of_polygons *new_cells = new list_of_polygons();
	// Connection_map connections;
	for (int i = 0; i < cells.size(); i++)
	{
		polygon *p = boost_polygon_to_polygon(cells[i]);
		if (p != NULL)
		{
			p->id = "CELL_" + to_string(i); // Set id of polygon
			p->area = bg::area(cells[i]);	// Set area of the cell

			connections.add_element(p);
			if (connections.connections[p->id].master == NULL)
			{
				cout << p->id << " is NULL!" << endl;
			}
		}
	};

	connections.aggregate();
	// connections.info();
	connections.ensure_LOS(obstacles);

	// Populate free space
	map<string, polygon *> els = connections.elements();
	map<string, polygon *>::const_iterator new_it;

	for (new_it = els.cbegin(); new_it != els.cend(); new_it++)
	{
		free_space->add_polygon(new_it->second);
	}
	connections.info();
};

/**
 * This function is used to detect the free space in the arena provided.
 * The algorithm works by subtracting to the arena the obstacles identified
 * in it.
 * The arena is cut into several cells which have the arena centroid as the
 * reference point. Then the obstacles are subtracted from those cells
 * retaining only the free space in it.
 * @param[in] res: int. Is the resolution to apply at the arena subsetting.
 */
void points_map::make_free_space_cells_triangular(int res)
{
	// Subset arena -> free space idealization
	polygon *_arena = new polygon(shrinked_arena);
	list_of_polygons *_arena_subset = subset_polygon(_arena, res);
	log->add_event("Arena subsetted");

	// Arena subsets to boost::polygons;
	vector<Polygon_boost> arena_polys;
	vector<Polygon_boost> arena_obstacles;
	vector<Polygon_boost> output;
	vector<Polygon_boost> tmp_output;

	polygon *pol = _arena_subset->head;
	while (pol != NULL)
	{
		arena_polys.push_back(pol->to_boost_polygon());
		pol = pol->pnext;
	};
	log->add_event("Arena converted to boost");

	pol = obstacles->offset_head;
	while (pol != NULL)
	{
		arena_obstacles.push_back(pol->to_boost_polygon());
		pol = pol->pnext;
	};
	log->add_event("Obstacles converted to boost");

	pol = gates->head;
	while (pol != NULL)
	{
		arena_obstacles.push_back(pol->to_boost_polygon());
		pol = pol->pnext;
	};
	log->add_event("Arena gates added to obstacles list");

	// Remove the obstacles from the free space and compute the new shapes
	if (arena_obstacles.size() > 0)
	{
		// Remove obstacles from the sectors of the arena.
		arena_polys = difference_of_vectors(arena_polys, arena_obstacles);

		// Remove occlusions caused by the free space with themselves
		int prev_output = output.size();
		int arena_size = arena_polys.size();
		int arena_ob_size = arena_obstacles.size();

		output.clear();
		tmp_output.clear();
		prev_output = 0;
		for (int i = 0; i < arena_size; i++)
		{
			for (int j = 0; j < arena_size; j++)
			{
				if (i != j)
				{
					if (bg::overlaps(arena_polys[i], arena_polys[j]))
					{
						bg::difference(arena_polys[i], arena_polys[j], output);
						int diff = output.size() - prev_output;
						prev_output = output.size();

						// printf("Polygon_boost %d and polygon %d intersects", i, j);
						// printf(" -> %d new cells\n", diff);

						arena_polys[i] = output[output.size() - 1];
						if (diff > 1)
						{
							tmp_output = arena_polys;
							for (int k = 1; k < diff; k++)
							{
								int output_idx = output.size() - 1 - k;
								vector<Polygon_boost>::iterator it;
								it = tmp_output.begin();
								tmp_output.insert(it + i + k, output[output_idx]);
								arena_size += 1;
							};
							arena_polys = tmp_output;
						};
					};
				};
			};
		};
	};

	output = arena_polys;

	// convert boost polygons to polygons and update free space variable
	for (int i = 0; i < output.size(); i++)
	{
		point_list *new_space_points = boost_polygon_to_point_list(output[i]);
		polygon *new_space = new polygon(new_space_points);
		free_space->add_polygon(new_space);
	};
	log->add_event("Free space generated");
};

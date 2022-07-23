#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "Roadmap/roadmap.h"
#include "Dubins/dubins.h"
#include "World_representation/world_representation.h"

#include <stdexcept>
#include <sstream>
#include <thread>
#include <time.h>

void thread_fugitive_plan(map<string, robot_fugitive *>::iterator,
                          World_representation);
void thread_catcher_plan(map<string, robot_catcher *>::iterator,
                         World_representation, behaviour_fugitive, bool);

/*
PLANNER https://fai.cs.uni-saarland.de/hoffmann/ff/FF-v2.3.tgz
HOME_PAGE: https://fai.cs.uni-saarland.de/hoffmann/ff.html

pre-requisite: flex e bison
*/

namespace student
{

  void loadImage(cv::Mat &img_out, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED");
  }

  void genericImageListener(const cv::Mat &img_in, std::string topic, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - IMAGE LISTENER - NOT CORRECTLY IMPLEMENTED");
  }

  bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix, cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED");
  }

  void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out,
                      const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, const std::string &config_folder)
  {

    throw std::logic_error("STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED");
  }

  void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec,
                          const cv::Mat &tvec, const std::vector<cv::Point3f> &object_points_plane,
                          const std::vector<cv::Point2f> &dest_image_points_plane,
                          cv::Mat &plane_transf, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED");
  }

  void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf,
              const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED");
  }

  bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacle_list, std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED");
  }

  bool findRobot(const cv::Mat &img_in, const double scale, Polygon &triangle, double &x, double &y, double &theta, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED");
  }

  bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list,
                const std::vector<Polygon> &gate_list, const std::vector<float> x,
                const std::vector<float> y, const std::vector<float> theta,
                std::vector<Path> &path, const std::string &config_folder)
  {
    bool push_first = false;
    clock_t starting_clock = clock();

    logger *log_test = new logger("test_log.txt");
    log_test->add_event("Code started");
    points_map arena(log_test);

    /* Add arena limits */
    point_list *arena_limits = new point_list;
    for (int i = 0; i < borders.size(); i++)
    {
      arena_limits->add_node(new point_node(borders[i].x, borders[i].y));
    }
    arena.add_arena_points(arena_limits);

    /* Add obstacles */
    point_list *pol;
    for (int i = 0; i < obstacle_list.size(); i++)
    {
      pol = new point_list;
      for (int j = 0; j < obstacle_list[i].size(); j++)
      {
        pol->add_node(new point_node(obstacle_list[i][j].x, obstacle_list[i][j].y));
      }
      arena.add_obstacle(new polygon(pol));
    }

    /* Add gates */
    for (int i = 0; i < gate_list.size(); i++)
    {
      pol = new point_list;
      for (int j = 0; j < gate_list[i].size(); j++)
      {
        pol->add_node(new point_node(gate_list[i][j].x, gate_list[i][j].y));
      }
      arena.add_gate(new polygon(pol));
    }

    // Before merging obstacle they must be convexify to avoid boost::error
    // arena.make_free_space_cells_squares(RES);
    arena.make_exact_cell();

    cout << "Made free space" << endl;
    
    log_test->add_event("Created Roadmap");

    Robot *c_1 = new Robot("Catcher_1", catcher, log_test);
    arena.add_robot(c_1);

    Robot *f_1 = new Robot("Fugitive_1", fugitive, log_test);
    arena.add_robot(f_1);

    arena.set_robot_position(f_1->ID, x[0], y[0], theta[0]);
    arena.set_robot_position(c_1->ID, x[1], y[1], theta[1]);
    
    // Create world representaion
    World_representation abstract_arena = World_representation(
        log_test,
        arena.free_space,
        arena.gates,
        arena.obstacles,
        &arena.connections,
        arena.connections.find_pddl_connections(),
        arena.connections.make_cells_predicates(),
        arena.connections.make_cells_conditional_distances());
    
    robot_manager rm(log_test);

    rm.add_robot(f_1);
    rm.add_robot(c_1);

    rm.trade_fugitives();

    // rm.info(true);

    abstract_arena.info();
    Mat img_arena = arena.plot_arena(1080, 1080, true, PLOT_CELL_ID);
    // imshow("Arena", img_arena);
    // waitKey(0);

    /**********************************************
     * GENERATE PLANS AND MOVE ROBOTS
     ***********************************************/
    
    map<string, robot_fugitive *>::iterator f_it;
    f_it = rm.fugitives.begin();
    f_it->second->set_behaviour(aware);

    map<string, robot_catcher *>::iterator c_it;
    c_it = rm.catchers.begin();

    // Show plans of the robots
    rm.run_agents_planners(abstract_arena, aware);
    rm.info(true);
      
    // imshow("Arena", img_arena);
    // imwrite("Arena.png", img_arena);
    // waitKey(0);

    string location_f = find_agent_location_pddl(f_1, abstract_arena, true);
    string location_c = find_agent_location_pddl(c_1, abstract_arena, true);

    vector<curve> f_path;
    // Get the dubins path without intersection
    // if(location_f.compare(location_c) != 0)
    // {
    //   f_path = get_dubins_path(arena, abstract_arena, f_1);
    // }
    f_path = get_dubins_path(arena, abstract_arena, f_1);
    vector<curve> c_path = get_dubins_path(arena, abstract_arena, c_1);;
   
    /*
      Push only the first action move to the simulator.
      In this way I have to run the simulation multiple times
      and make a plan every time.
     */
   
    if (push_first)
    {
      if (f_path.size() != 0)
        path[0] = push_path(f_path[0], path[0]);
      if (c_path.size() != 0)
        path[1] = push_path(c_path[0], path[1]);
    }
    else
    {

      for (int i = 0; i < f_path.size(); i++)
      {
        path[0] = push_path(f_path[i], path[0]);
        img_arena = plotdubins(f_path[i], "r", "r", "r", img_arena);
      }
      for (int i = 0; i < c_path.size(); i++)
      {
        path[1] = push_path(c_path[i], path[1]);
        img_arena = plotdubins(c_path[i], "y", "y", "y", img_arena);
      }
    }

    float elapsed_time = (float(clock() - starting_clock)) / CLOCKS_PER_SEC;
    log_test->add_event("Code ended in " + to_string(elapsed_time) + "s\n");
    
    if (!push_first)
    {
      imshow("Arena", img_arena);
      imwrite("Arena.png", img_arena);
      waitKey(0);
    }

    return true;
  }
}

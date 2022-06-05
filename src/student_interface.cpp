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
    // throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );

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

    arena.merge_obstacles();

    arena.make_free_space_cells_squares();
    // arena.make_free_space_cells_triangular();

    log_test->add_event("Created Roadmap");

    Robot *c_1 = new Robot("Catcher_1", catcher, log_test);
    arena.add_robot(c_1);

    Robot *f_1 = new Robot("Fugitive_1", fugitive, log_test);
    arena.add_robot(f_1);

    arena.set_robot_position(f_1->ID, x[0], y[0], theta[0]);
    arena.set_robot_position(c_1->ID, x[1], y[1], theta[1]);

    // Create world representaion
    World_representation abstract_arena = World_representation(
        arena.free_space,
        arena.gates,
        log_test);
    abstract_arena.find_pddl_connections();

    robot_manager rm(log_test);

    rm.add_robot(f_1);
    rm.add_robot(c_1);

    rm.trade_fugitives();

    // rm.info(true);

    abstract_arena.info();
    Mat img_arena = arena.plot_arena(600, 600, true);

    /**********************************************
     * GENERATE PLANS AND MOVE ROBOTS
     ***********************************************/

    // FUGITIVE PLAN

    map<string, robot_fugitive *>::iterator f_it;
    f_it = rm.fugitives.begin();

    map<string, robot_catcher *>::iterator c_it;
    c_it = rm.catchers.begin();
    // c_it->second->make_pddl_files(abstract_arena, f_it->second->behaviour, true);

    thread f_thr(thread_fugitive_plan, f_it, abstract_arena);
    sleep(1);
    thread c_thr(thread_catcher_plan, c_it, abstract_arena,
                 f_it->second->behaviour, true);

    // thread_fugitive_plan(f_it, abstract_arena);
    // thread_catcher_plan(c_it, abstract_arena, f_it->second->behaviour, true);
    f_thr.join();
    c_thr.join();

    // Show plans of the robots
    rm.info(true);

    /* Fugitive  path vectors */
    vector<double> fx_path;
    vector<double> fy_path;
    vector<double> fth_path; // The angles are in radiants!

    clock_t tStart = clock();

    /* Get the cells centroids of the fugitive path */
    tie(fx_path, fy_path) = abstract_arena.get_path(f_it->second->self->plan);

    /*Get the starting angle for moving from a cell to another one */
    fth_path = opti_theta(fx_path, fy_path);
    
    fx_path[0] = f_it->second->self->location->x;
    fy_path[0] = f_it->second->self->location->y;
    fth_path[0] = f_1->theta; 

    curve c;
    // // double kmax = 36;
    double kmax = 39;
    // double kmax = 41;
    double search_angle = M_PI * 2;

    /* Calculate fugitive's dubin curves without intersection */
    for (int i = 0; i < fx_path.size() - 1; i++)
    {
      c = dubins_no_inter(fx_path[i], fy_path[i], fth_path[i], fx_path[i + 1],
                          fy_path[i + 1], &fth_path[i + 1], kmax, arena,
                          search_angle);

      /*
        Push only the first action move to the simulator.
        In this way I have to run the simulation multiple times
        and make a plan every time.
       */
      if (push_first)
      {
        if (i == 0)
          path[0] = push_path(c, path[0]);
      }
      else
        path[0] = push_path(c, path[0]);

      img_arena = plotdubins(c, "r", "r", "r", img_arena);
    }
    printf("Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

    // CATCHER PLAN

    vector<double> cx_path;
    vector<double> cy_path;
    vector<double> cth_path;

    tie(cx_path, cy_path) = abstract_arena.get_path(c_it->second->self->plan);

    cth_path = opti_theta(cx_path, cy_path);

    cx_path[0] = c_it->second->self->location->x;
    cy_path[0] = c_it->second->self->location->y;
    cth_path[0] = c_1->theta; 

    for (int i = 0; i < cx_path.size() - 1; i++)
    {
      c = dubins_no_inter(cx_path[i], cy_path[i], cth_path[i], cx_path[i + 1],
                          cy_path[i + 1], &cth_path[i + 1], kmax, arena,
                          search_angle);

      if (push_first)
      {
        if (i == 0)
          path[1] = push_path(c, path[1]);
      }
      else
        path[1] = push_path(c, path[1]);

      img_arena = plotdubins(c, "b", "b", "b", img_arena);
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

void thread_fugitive_plan(map<string, robot_fugitive *>::iterator f_it,
                          World_representation wr)
{
  f_it->second->set_behaviour(aware);
  f_it->second->make_pddl_domain_file();
  f_it->second->make_pddl_problem_file(wr);
}

void thread_catcher_plan(map<string, robot_catcher *>::iterator c_it,
                         World_representation wr,
                         behaviour_fugitive type, bool a)
{
  c_it->second->make_pddl_files(wr, type, a);
}

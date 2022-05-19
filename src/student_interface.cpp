#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "Roadmap/roadmap.h"
#include "Dubins/dubins.h"
#include "World_representation/world_representation.h"
#include <thread>

#include <stdexcept>
#include <sstream>

Pose get_pose(arc, bool last_elem = false);
Path push_path(curve, Path);

// Scaler factor for plotting arena with opencv
int scale = 1;

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

  bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list, const std::vector<Polygon> &gate_list, const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta, std::vector<Path> &path, const std::string &config_folder)
  {
    // throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );

    logger *log_test = new logger;
    log_test->set_log_path("test_log.txt");
    log_test->add_event("Code started\n");
    points_map arena(log_test);

    // Add arena limits
    point_list *arena_limits = new point_list;
    for (int i = 0; i < borders.size(); i++)
    {
      arena_limits->add_node(new point_node(borders[i].x * scale, borders[i].y * scale));
    }
    arena.add_arena_points(arena_limits);

    // Add obstacles
    point_list *pol;
    for (int i = 0; i < obstacle_list.size(); i++)
    {
      pol = new point_list;
      for (int j = 0; j < obstacle_list[i].size(); j++)
      {
        pol->add_node(new point_node(obstacle_list[i][j].x * scale, obstacle_list[i][j].y * scale));
      }
      arena.add_obstacle(new polygon(pol));
    }

    // Add gates
    for (int i = 0; i < gate_list.size(); i++)
    {
      pol = new point_list;
      for (int j = 0; j < gate_list[i].size(); j++)
      {
        pol->add_node(new point_node(gate_list[i][j].x * scale, gate_list[i][j].y * scale));
      }
      arena.add_gate(new polygon(pol));
    }

    arena.merge_obstacles();

    arena.make_free_space_cells_squares();
    log_test->add_event("Created Roadmap");

    Robot *c_1 = new Robot("Catcher_1", catcher);
    arena.add_robot(c_1);

    Robot *f_1 = new Robot("Fugitive_1", fugitive);
    arena.add_robot(f_1);

    arena.set_robot_position(f_1->ID, x[0] * scale, y[0] * scale, theta[0]);
    arena.set_robot_position(c_1->ID, x[1] * scale, y[1] * scale, theta[1]);
    // Create world representaion
    World_representation abstract_arena = World_representation(
        arena.free_space,
        arena.gates,
        log_test);

    robot_manager rm;

    rm.add_robot(f_1);
    rm.add_robot(c_1);

    rm.trade_fugitives();

    // rm.info(true);

    abstract_arena.info();

    map<string, robot_fugitive *>::iterator f_it;
    f_it = rm.fugitives.begin();
    f_it->second->set_behaviour(aware);
    f_it->second->make_pddl_domain_file(abstract_arena);
    f_it->second->make_pddl_problem_file(abstract_arena);

    // map<string, robot_catcher *>::iterator c_it;
    // c_it = rm.catchers.begin();
    // c_it->second->make_pddl_files(abstract_arena, f_it->second->behaviour, true);

    // it->second->info();

    Mat img_arena = arena.plot_arena(800, 800, true);

    /**********************************************
     * TO MOVE
     ***********************************************/

    vector<double> fx_path;
    vector<double> fy_path;
    vector<double> fth_path; // The angles are in radiants!

    tie(fx_path, fy_path) = abstract_arena.get_path(f_it->second->self->plan);

    fth_path = opti_theta(fx_path, fy_path);

    curve c;
    // Calculate dubin's curves without intersection

    c = dubins_no_inter(f_it->second->self->location->x, f_it->second->self->location->y, f_1->theta, fx_path[0], fy_path[0], &fth_path[0], 0, arena);
    path[0] = push_path(c, path[0]);

    // for(int i=0;i < path[0].points.size();i++)
    // {
    //   cout << path[0].points[0].x << endl;
    // }

    img_arena = plotdubins(c, "r", "g", "b", img_arena);

    for (int i = 0; i < fx_path.size() - 1; i++)
    {
      c = dubins_no_inter(fx_path[i], fy_path[i], fth_path[i], fx_path[i + 1], fy_path[i + 1], &fth_path[i + 1], 10, arena);
      path[0] = push_path(c, path[0]);

      img_arena = plotdubins(c, "r", "g", "b", img_arena);
    }

    // vector<double> cx_path;
    // vector<double> cy_path;
    // vector<double> cth_path; // The angles are in radiants!

    // tie(cx_path, cy_path) = abstract_arena.get_path(c_it->second->self->plan);

    // cth_path = opti_theta(cx_path, cy_path);

    // c = dubins_no_inter(c_it->second->self->location->x, c_it->second->self->location->y, c_1->theta, cx_path[0], cy_path[0], &cth_path[0], 0, arena);
    // img_arena = plotdubins(c, "r", "g", "b", img_arena);

    // for (int i = 0; i < cx_path.size() - 1; i++)
    // {
    //   c = dubins_no_inter(cx_path[i], cy_path[i], cth_path[i], cx_path[i + 1], cy_path[i + 1], &cth_path[i + 1], 10, arena);
    //   img_arena = plotdubins(c, "r", "g", "b", img_arena);
    // }
    // /**********************************************/

    // imshow("Arena", img_arena);
    // waitKey(0);

    return true;
  }
}


Pose get_pose(arc a, bool last_elem)
{
  Pose p;
  p.s = a.L;
  p.kappa = a.k;

  if (last_elem == true)
  {
    p.x = a.xf;
    p.y = a.yf;
    p.theta = a.thf;
    return p;
  }

  p.x = a.x0;
  p.y = a.y0;
  p.theta = a.th0;

  return p;
}

Path push_path(curve c, Path p)
{
  arc a;

  a = dubinsarc(c.a1.x0, c.a1.y0, c.a1.th0, c.a1.k, c.a1.L / 100);
  for (int i = 0; i < 100; i++)
  {
    p.points.push_back(get_pose(a));
    a = dubinsarc(a.xf, a.yf, a.thf, a.k, a.L);
  }
  p.points.push_back(get_pose(a, true));

  a = dubinsarc(c.a2.x0, c.a2.y0, c.a2.th0, c.a2.k, c.a2.L / 100);
  for (int i = 0; i < 100; i++)
  {
    p.points.push_back(get_pose(a));
    a = dubinsarc(a.xf, a.yf, a.thf, a.k, a.L);
  }
  p.points.push_back(get_pose(a, true));

  a = dubinsarc(c.a3.x0, c.a3.y0, c.a3.th0, c.a3.k, c.a3.L / 100);
  for (int i = 0; i < 100; i++)
  {
    p.points.push_back(get_pose(a));
    a = dubinsarc(a.xf, a.yf, a.thf, a.k, a.L);
  }
  p.points.push_back(get_pose(a, true));

  return p;
}
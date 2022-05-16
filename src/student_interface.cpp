#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "Roadmap/roadmap.h"
#include "Dubins/dubins.h"
#include "World_representation/world_representation.h"
#include <thread>

#include <stdexcept>
#include <sstream>

// Scaler factor for plotting arena with opencv
int scale = 2;

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
      cout << "Poligono " << i + 1 << "\n";
      for (int j = 0; j < obstacle_list[i].size(); j++)
      {
        pol->add_node(new point_node(obstacle_list[i][j].x * scale, obstacle_list[i][j].y * scale));
        cout << obstacle_list[i][j].x << ":" << obstacle_list[i][j].y << " ";
      }
      cout << "\n";
      arena.add_obstacle(new polygon(pol));
    }

    // Add gates
    for (int i = 0; i < gate_list.size(); i++)
    {
      pol = new point_list;
      for (int j = 0; j < gate_list[i].size(); j++)
      {
        pol->add_node(new point_node(gate_list[i][j].x * scale, gate_list[i][j].y * scale));
        cout << "p" << j + 1 << " " << gate_list[i][j].x << ":" << gate_list[i][j].y << endl;
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

    arena.set_robot_position(c_1->ID, x[0] * scale, y[0] * scale);
    arena.set_robot_position(f_1->ID, x[1] * scale, y[1] * scale);
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

    map<string, robot_fugitive *>::iterator it;
    it = rm.fugitives.begin();
    it->second->set_behaviour(aware);
    it->second->make_pddl_domain_file(abstract_arena);
    it->second->make_pddl_problem_file(abstract_arena);

    // it->second->info();

    Mat img_arena = arena.plot_arena(800, 800, true);

    // /**********************************************
    //  * TO MOVE
    // //  ***********************************************/
    vector<string> f_path = it->second->self->plan;
    // // // cout << abstract_arena.world_free_cells["Cell_1"].cell->centroid->y<<endl;

    double fx_path[f_path.size() + 1];
    double fy_path[f_path.size() + 1];
    double fth_path[f_path.size() + 1]; // Radiants!

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

      if (i == f_path.size() - 1)
      {
        fx_path[i + 1] = abstract_arena.world_gates[path[3]].cell->centroid->x;
        fy_path[i + 1] = abstract_arena.world_gates[path[3]].cell->centroid->y;
      }
    }

    opti_theta(fx_path, fy_path, fth_path, f_path.size() + 1);

    curve c;

    // Calculate dubin's curves without intersection
  
    cout << "Curve: " << 0 << endl;
    c = dubins_no_inter(it->second->self->location->x, it->second->self->location->y, 0, fx_path[0], fy_path[0], fth_path[0], 0, arena);
    img_arena = plotdubins(c, "r", "g", "b", img_arena);

    for (int i = 0; i < f_path.size(); i++)
    {
      cout << "Curve: " << i+1 << endl;
      c = dubins_no_inter(fx_path[i], fy_path[i], fth_path[i], fx_path[i + 1], fy_path[i + 1], fth_path[i + 1], 10, arena);
      img_arena = plotdubins(c, "r", "g", "b", img_arena);
    }
    /**********************************************/
    imshow("Arena", img_arena);
    waitKey(0);

    return true;
  }
}

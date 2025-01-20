#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "config.h"
#include "map.hpp"
#include "object.hpp"
#include "utils/astar.hpp"
#include "utils/topp.hpp"

/**
 *
 * @param argv[0]: node name
 * @param argv[1]: have algae, "A" means have algae, "N" means no algae
 * @param argv[2]: have coral, "C" means have coral, "N" means no coral
 * @param argv[3]: begin t, double
 * @param argv[4]: begin r, double
 * @param argv[5]: end t, double
 * @param argv[6]: end r, double
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, argv[0]);
  ros::NodeHandle nh;
  ros::Rate rate(2000);

  bool haveAlgae = argv[1] == "A";
  bool haveCoral = argv[2] == "C";
  double begin_t = atof(argv[3]);
  double begin_r = atof(argv[4]);
  double end_t = atof(argv[5]);
  double end_r = atof(argv[6]);

  // publishers & subscribers
  ros::Publisher cspace_pub =
      nh.advertise<nav_msgs::GridCells>("/cyber_planner/cspace", 10);
  ros::Publisher espace_pub =
      nh.advertise<nav_msgs::GridCells>("/cyber_planner/espace", 10);
  ros::Publisher bound_pub =
      nh.advertise<nav_msgs::GridCells>("/cyber_planner/bound", 10);
  ros::Publisher points_pub =
      nh.advertise<visualization_msgs::Marker>("/cyber_planner/points", 10);
  ros::Publisher path_pub =
      nh.advertise<visualization_msgs::Marker>("/cyber_planner/path", 10);
  ros::Publisher plot_at_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/path/plot/at", 10);
  ros::Publisher plot_ax_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/path/plot/ax", 10);
  ros::Publisher plot_ay_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/path/plot/ay", 10);
  ros::Publisher plot_vt_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/path/plot/vt", 10);
  ros::Publisher plot_vx_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/path/plot/vx", 10);
  ros::Publisher plot_vy_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/path/plot/vy", 10);

  Object env = Object();
  Object arm_espace = Object(true, haveAlgae, haveCoral);
  Object arm_cspace = Object(false, haveAlgae, haveCoral);
  ROS_INFO("arm & env initialized, have algae: %d, have coral: %d", haveAlgae,
           haveCoral);

  std::vector<std::vector<bool>> map_espace, map_cspace, map_bound;
  getGridMap(env, arm_espace, map_espace);
  getGridMap(env, arm_cspace, map_cspace);

  nav_msgs::GridCells cspace_msg, espace_msg, bound_msg;
  visualization_msgs::Marker points_msg;

  renderMap(espace_msg, map_espace, 0);
  renderMap(cspace_msg, map_cspace, -1);
  renderPoints(points_msg, std::vector<Eigen::Vector2d>{{begin_t, begin_r},
                                                        {end_t, end_r}});
  renderBound(bound_msg);

  Eigen::Vector2i begin_idx = getGridIdx(begin_t, begin_r);
  Eigen::Vector2i end_idx = getGridIdx(end_t, end_r);
  ROS_INFO("grid map initialized, begin: (%d, %d), end: (%d, %d)", begin_idx(0),
           begin_idx(1), end_idx(0), end_idx(1));

  std::vector<Eigen::Vector2i> path, visited;
  bool found = astar::astar(map_espace, begin_idx, end_idx, path, visited);
  if (!found) {
    ROS_WARN("no path found in espace, try cspace");
    if (!astar::astar(map_cspace, begin_idx, end_idx, path, visited)) {
      ROS_ERROR("no path found");
      return 0;
    }
  }

  visualization_msgs::Marker path_msg;
  renderPath(path_msg, getTRs(path));
  ROS_INFO("path found");

  Topp topp(getTRs(path));
  ROS_INFO(
      "loss goes up is normal, "
      "due to the penalty coefficient is getting larger");
  while (!topp.step()) {
    ROS_INFO("iter: %d, loss: %f", topp.getIter(), topp.getLoss());
  }
  ROS_INFO("topp finished");

  std::vector<double> timestamp;
  std::vector<Eigen::Vector2d> velocity, acceleration;
  topp.get(timestamp, velocity, acceleration);
  std_msgs::Float64MultiArray plot_at_msg, plot_ax_msg, plot_ay_msg,
      plot_vt_msg, plot_vx_msg, plot_vy_msg;

  // TODO: fix
  int sim_idx = 0;
  double sim_t = 0;
  plot_vt_msg.data.push_back(sim_t);
  plot_vx_msg.data.push_back(velocity[sim_idx](0));
  plot_vy_msg.data.push_back(velocity[sim_idx](1));
  while (sim_idx <= timestamp.size()) {
    plot_at_msg.data.push_back(sim_t);
    plot_ax_msg.data.push_back(acceleration[sim_idx](0));
    plot_ay_msg.data.push_back(acceleration[sim_idx](1));
    if (sim_t >= timestamp[sim_idx + 1]) {
      plot_vt_msg.data.push_back(sim_t);
      plot_vx_msg.data.push_back(velocity[sim_idx](0));
      plot_vy_msg.data.push_back(velocity[sim_idx](1));
      ++sim_idx;
    }
    sim_t += config::DT;
  }
  ROS_INFO("plot msg obtained");

  while (ros::ok()) {
    cspace_pub.publish(cspace_msg);
    espace_pub.publish(espace_msg);
    bound_pub.publish(bound_msg);
    points_pub.publish(points_msg);
    path_pub.publish(path_msg);
    plot_at_pub.publish(plot_at_msg);
    plot_ax_pub.publish(plot_ax_msg);
    plot_ay_pub.publish(plot_ay_msg);
    plot_vt_pub.publish(plot_vt_msg);
    plot_vx_pub.publish(plot_vx_msg);
    plot_vy_pub.publish(plot_vy_msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
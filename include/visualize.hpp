// #include <matplot/matplot.h>

// #include <Eigen/Eigen>

// #include "config.h"
// #include "log.hpp"
// #include "map.hpp"

// using namespace config::alphabot;

// void visualizeTrajectory(const std::vector<std::vector<double>>& armGrid,
//                          const std::vector<std::vector<double>>& expGrid,
//                          const std::vector<Eigen::Vector2d>& path) {
//   using namespace matplot;

//   log_info("Visualizing the trajectory...");

//   auto x = linspace(0, 3 * pi, 200);
//   auto y = transform(x, [&](double x) { return cos(x) + rand(0, 1); });
//   auto c = linspace(1, 10, x.size());

//   scatter(x, y, std::vector<double>{}, c);

//   //   for (int tt = 0; tt < armGrid.size(); tt++) {
//   //     for (int rr = 0; rr < armGrid[tt].size(); rr++) {
//   //       Eigen::Vector2d tr = getTR(tt, rr);
//   //       double t = tr.x(), r = tr.y();
//   //       if (armGrid[tt][rr] > config::params::OBSTACLE_OFFSET - .01) {
//   //         auto s = scatter(t, r, 6);
//   //         s->marker_face_color("r");
//   //         s->marker_color("r");
//   //       } else if (expGrid[tt][rr] > config::params::OBSTACLE_OFFSET - .01) {
//   //         auto s = scatter(t, r, 6);
//   //         s->marker_face_color("b");
//   //         s->marker_color("b");
//   //       }
//   //     }
//   //   }

//   //   show();
// }
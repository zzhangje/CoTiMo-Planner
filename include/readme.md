### Math Utils:

- `lbfgs.hpp`: Use L-BFGS Method to solve unconstrained optimized problem, https://github.com/ZJU-FAST-Lab/LBFGS-Lite
- `sdqp.hpp`: Use SDQP Method to solve low dimensional linear inequality constraints quadratically optimization problem, https://github.com/ZJU-FAST-Lab/SDQP
- `astar.hpp`: A* path finding
- `spline.hpp`: obtain the params of a spline by given points
- `Topp.hpp`: form trajectory planning problem to PHR Augumented Lagrangian Relaxation form, and solve it with L-BFGS
- `Smooth.hpp`: form path optimization problem to unconstrained optimization form and solve it with L-BFGS

### System Utils:

- `log.hpp`, https://github.com/rxi/log.c

### Object Class:

- `Polygon.hpp`: define some basic properties for polygon object
- `Object.hpp`: every object can be formed by finite number of polygons. environment obstacles, arm, and expanded arm are all objects.
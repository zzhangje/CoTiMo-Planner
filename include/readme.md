## Libs & Utils Class

| Type | File | Description | Reference |
| :-: | :-: | - | - |
| Object | [libs/Polygon.hpp](libs/Polygon.hpp) | Define the basic properties of a polygon:<br>`isPointInside`, `isPolygonIntersect`, `distanceToPoint`, `distanceToPolygon` | |
| Object | [libs/Object.hpp](libs/Object.hpp) | Any object (environment obstacle, arm) can be represented as a finite set of convex polygons  | |
| Task | [libs/Smooth.hpp](libs/Smooth.hpp) | Establish an unconstrained optimization problem under the Lipschitz continuous artificial potential field to minimize stretch energy | |
| Task | [libs/Topp.hpp](libs/Topp.hpp) | Relax the time-optimal parameterization problem of kinematic constraints into a PHR Augumented Lagrangian optimization problem  | |
| Math Util | [libs/lbfgs.hpp](libs/lbfgs.hpp) | Solve unconstrained optimization problem with L-BFGS algorithm | https://github.com/ZJU-FAST-Lab/LBFGS-Lite |
| Math Util | [libs/sdqp.hpp](libs/sdqp.hpp) | Solve linear inequality constrained quadratically optimization problem with LDQP algorithm | https://github.com/ZJU-FAST-Lab/SDQP |
| Math Util | [libs/spline.hpp](libs/spline.hpp) | Given a series of points, calculate the polynomial curve of the specified order | |
| System Util | [utils/log.hpp](utils/log.hpp) | | https://github.com/rxi/log.c |

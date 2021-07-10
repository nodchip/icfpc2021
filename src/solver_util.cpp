#include "stdafx.h"
#include "solver_util.h"
#include "contest_types.h"

Point find_collapse_point(SProblemPtr problem) {
  integer xmin = 0, xmax = 0, ymin = 0, ymax = 0;
  bounding_box(problem->hole_polygon, xmin, xmax, ymin, ymax);

  for (integer x = xmin; x <= xmax; x+=std::max<integer>(1, (xmax - xmin) / 100)) {
    for (integer y = ymin; y <= ymax; y+=std::max<integer>(1, (ymax - ymin) / 100)) {
    }
  }
}

void bounding_box(const std::vector<Point>& points, integer& xmin, integer& xmax, integer& ymin, integer& ymax) {
  integer ymin = INT_MAX, ymax = INT_MIN;
  integer xmin = INT_MAX, xmax = INT_MIN;
  for (auto p : points) {
    xmin = std::min(xmin, get_x(p));
    ymin = std::min(ymin, get_y(p));
    xmax = std::max(xmax, get_x(p));
    ymax = std::max(ymax, get_y(p));
  }
}

#include <kimera_rviz_markers/utils.h>

#include <cstdlib>

namespace krm {

double rand(double min, double max) {
  double t = static_cast<double>(std::rand()) /
      static_cast<double>(RAND_MAX);
  return min + t*(max-min);
}

}


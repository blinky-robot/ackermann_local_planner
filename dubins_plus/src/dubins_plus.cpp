/*
 * Implementation of Dubins path and similar algorithms
 */

#include "dubins_plus/dubins_plus.h"

// SHUT UP BOOST SIGNALS
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <tf/tf.h>
#include <cmath>

namespace dubins_plus {
  std::vector<Segment> dubins_path(double radius,
      double x, double y, double theta) {
    std::vector<Segment> result;
    // See: http://planning.cs.uiuc.edu/node821.html
    // TODO(hendrix): MAGIC!
    return result;
  }

  std::vector<Segment> dubins_path(double radius,
      double x1, double y1, double theta1,
      double x2, double y2, double theta2) {
    // TODO: normalize and call dubins_path(r, x, y, t)
    double x = x2 - x1;
    double y = y2 - y1;
    double theta = theta2 - theta1;
    // normalize theta
    while( theta > M_PI ) {
      theta -= 2*M_PI;
    }
    while( theta < -M_PI ) {
      theta += 2*M_PI;
    }
    return dubins_path(radius, x, y, theta);
  }

  std::vector<Segment> dubins_path(double radius,
      geometry_msgs::Pose &start, geometry_msgs::Pose &end) {
    return dubins_path(radius,
        start.position.x, start.position.y, tf::getYaw(start.orientation),
        end.position.x, end.position.y, tf::getYaw(end.orientation));
  }
};


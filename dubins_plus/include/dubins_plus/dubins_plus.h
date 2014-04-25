/**
 * dubins_plus: a library for Dubins' path and similar algorithms
 *
 * Intended mostly for interface with raw data and ROS data types,
 * particularly for use in planners
 *
 * See: http://planning.cs.uiuc.edu/node820.html
 *  - Dubins: http://planning.cs.uiuc.edu/node821.html
 *  - Reeds-Shepp: http://planning.cs.uiuc.edu/node822.html
 *  - Balkcom-Mason: http://planning.cs.uiuc.edu/node823.html
 *
 * Author: Austin Hendrix
 */

#ifndef DUBINS_PLUS_H
#define DUBINS_PLUS_H

#include <vector>
#include <geometry_msgs/Pose.h>

namespace dubins_plus {
  /**
   * @brief A class representing a segment of a path, consisting of a
   * length and a curvature
   */
  class Segment {
    public:
      /**
       * @brief Get the length of this segment. Nominally in meters.
       * Length is + for forwards, - for backwards
       */
      double getLength()    const { return length; }
      /**
       * @brief Get the curvature of this segment.
       * curvature = 1/radius, but is signed to indicate the direction of
       * the turn. + for left (counterclockwise), - for right (clockwise)
       */
      double getCurvature() const { return curvature; }

      /**
       * @brief Create a new segment with the given curvature and length
       */
      Segment(double length, double curvature) : length(length),
        curvature(curvature) {};
    private:
      double length;
      double curvature;
  };

  // the core algorithm: compute the path from the origin to the point given by
  // x,y,theta using segments of radius 1
  std::vector<Segment> dubins_path(double x, double y, double theta);

  // compute the path from the origin to the point given by
  // x,y,theta using segments of the given radius
  std::vector<Segment> dubins_path(double radius,
      double x, double y, double theta);

  // variant that takes start and end points
  std::vector<Segment> dubins_path(double radius,
      double x1, double y1, double theta1,
      double x2, double y2, double theta2);

  // variant that takes start and end Poses
  std::vector<Segment> dubins_path(double radius,
      geometry_msgs::Pose &start, geometry_msgs::Pose &end);

  // TODO(hendrix): Reeds-Shepp curves
  // TODO(hendrix): Balkcom-Mason curves
}; // namespace dubins_plus

#endif

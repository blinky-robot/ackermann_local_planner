/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* blatantly borrowed from OMPL; originally written by Mark Moll
 * Author: Austin Hendrix
 *
 * Implementation of Dubins path and similar algorithms
 */

#include "dubins_plus/dubins_plus.h"

// SHUT UP BOOST SIGNALS
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <tf/tf.h>
#include <cmath>

namespace dubins_plus {
#define TWO_PI (2*M_PI)
#define DUBINS_EPS (1e-6)
#define DUBINS_ZERO (-1e-9)
  // conveninet functions
  inline double mod2pi(double x) {
    if (x<0 && x>DUBINS_ZERO) return 0;
    return x - TWO_PI * floor(x / TWO_PI);
  }

  enum DubinsPathSegmentType { DUBINS_LEFT=0, DUBINS_STRAIGHT=1, DUBINS_RIGHT=2 };
  DubinsPathSegmentType dubinsPathType[6][3] = {
    { DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT },
    { DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT },
    { DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT },
    { DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT },
    { DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT },
    { DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT }
  };

  class DubinsPath
  {
    public:
      DubinsPath(const DubinsPathSegmentType* type = dubinsPathType[0],
          double t=0., double p=std::numeric_limits<double>::max(), double q=0.)
        : type_(type), reverse_(false)
      {
        length_[0] = t;
        length_[1] = p;
        length_[2] = q;
        assert(t >= 0.);
        assert(p >= 0.);
        assert(q >= 0.);
      }
      double length() const
      {
        return length_[0] + length_[1] + length_[2];
      }

      const DubinsPathSegmentType* type_;
      double length_[3];
      bool reverse_;
  };

  DubinsPath dubinsLSL(double d, double alpha, double beta)
  {
    double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
    double tmp = 2. + d*d - 2.*(ca*cb +sa*sb - d*(sa - sb));
    if (tmp >= DUBINS_ZERO)
    {
      double theta = atan2(cb - ca, d + sa - sb);
      double t = mod2pi(-alpha + theta);
      double p = sqrt(std::max(tmp, 0.));
      double q = mod2pi(beta - theta);
      assert(fabs(p*cos(alpha + t) - sa + sb - d) < DUBINS_EPS);
      assert(fabs(p*sin(alpha + t) + ca - cb) < DUBINS_EPS);
      assert(mod2pi(alpha + t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
      return DubinsPath(dubinsPathType[0], t, p, q);
    }
    return DubinsPath();
  }

  DubinsPath dubinsRSR(double d, double alpha, double beta)
  {
    double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
    double tmp = 2. + d*d - 2.*(ca*cb + sa*sb - d*(sb - sa));
    if (tmp >= DUBINS_ZERO)
    {
      double theta = atan2(ca - cb, d - sa + sb);
      double t = mod2pi(alpha - theta);
      double p = sqrt(std::max(tmp, 0.));
      double q = mod2pi(-beta + theta);
      assert(fabs(p*cos(alpha - t) + sa - sb - d) < DUBINS_EPS);
      assert(fabs(p*sin(alpha - t) - ca + cb) < DUBINS_EPS);
      assert(mod2pi(alpha - t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
      return DubinsPath(dubinsPathType[1], t, p, q);
    }
    return DubinsPath();
  }

  DubinsPath dubinsRSL(double d, double alpha, double beta)
  {
    double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
    double tmp = d * d - 2. + 2. * (ca*cb + sa*sb - d * (sa + sb));
    if (tmp >= DUBINS_ZERO)
    {
      double p = sqrt(std::max(tmp, 0.));
      double theta = atan2(ca + cb, d - sa - sb) - atan2(2., p);
      double t = mod2pi(alpha - theta);
      double q = mod2pi(beta - theta);
      assert(fabs(p*cos(alpha - t) - 2. * sin(alpha - t) + sa + sb - d) < DUBINS_EPS);
      assert(fabs(p*sin(alpha - t) + 2. * cos(alpha - t) - ca - cb) < DUBINS_EPS);
      assert(mod2pi(alpha - t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
      return DubinsPath(dubinsPathType[2], t, p, q);
    }
    return DubinsPath();
  }

  DubinsPath dubinsLSR(double d, double alpha, double beta)
  {
    double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
    double tmp = -2. + d * d + 2. * (ca*cb + sa*sb + d * (sa + sb));
    if (tmp >= DUBINS_ZERO)
    {
      double p = sqrt(std::max(tmp, 0.));
      double theta = atan2(-ca - cb, d + sa + sb) - atan2(-2., p);
      double t = mod2pi(-alpha + theta);
      double q = mod2pi(-beta + theta);
      assert(fabs(p*cos(alpha + t) + 2. * sin(alpha + t) - sa - sb - d) < DUBINS_EPS);
      assert(fabs(p*sin(alpha + t) - 2. * cos(alpha + t) + ca + cb) < DUBINS_EPS);
      assert(mod2pi(alpha + t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
      return DubinsPath(dubinsPathType[3], t, p, q);
    }
    return DubinsPath();
  }

  DubinsPath dubinsRLR(double d, double alpha, double beta)
  {
    double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
    double tmp = .125 * (6. - d * d  + 2. * (ca*cb + sa*sb + d * (sa - sb)));
    if (fabs(tmp) < 1.)
    {
      double p = TWO_PI - acos(tmp);
      double theta = atan2(ca - cb, d - sa + sb);
      double t = mod2pi(alpha - theta + .5 * p);
      double q = mod2pi(alpha - beta - t + p);
      assert(fabs( 2.*sin(alpha - t + p) - 2. * sin(alpha - t) - d + sa - sb) < DUBINS_EPS);
      assert(fabs(-2.*cos(alpha - t + p) + 2. * cos(alpha - t) - ca + cb) < DUBINS_EPS);
      assert(mod2pi(alpha - t + p - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
      return DubinsPath(dubinsPathType[4], t, p, q);
    }
    return DubinsPath();
  }

  DubinsPath dubinsLRL(double d, double alpha, double beta)
  {
    double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
    double tmp = .125 * (6. - d * d  + 2. * (ca*cb + sa*sb - d * (sa - sb)));
    if (fabs(tmp) < 1.)
    {
      double p = TWO_PI - acos(tmp);
      double theta = atan2(-ca + cb, d + sa - sb);
      double t = mod2pi(-alpha + theta + .5 * p);
      double q = mod2pi(beta - alpha - t + p);
      assert(fabs(-2.*sin(alpha + t - p) + 2. * sin(alpha + t) - d - sa + sb) < DUBINS_EPS);
      assert(fabs( 2.*cos(alpha + t - p) - 2. * cos(alpha + t) + ca - cb) < DUBINS_EPS);
      assert(mod2pi(alpha + t - p + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
      return DubinsPath(dubinsPathType[5], t, p, q);
    }
    return DubinsPath();
  }

  std::vector<Segment> dubins_path(double x, double y, double theta) {
    // See: http://planning.cs.uiuc.edu/node821.html
    // and: http://ftp.laas.fr/pub/ria/promotion/chap3.pdf (page 141)
    // and http://ompl.kavrakilab.org/DubinsStateSpace_8cpp_source.html
    // TODO(hendrix): MAGIC!
    double d  = sqrt(x*x + y*y);
    double th = atan2(y, x);
    double alpha = mod2pi(-th);
    double beta  = mod2pi(theta - th);

    // try all six primitives and pick the best:
    DubinsPath path(dubinsLSL(d, alpha, beta)), tmp(dubinsRSR(d, alpha, beta));
    double len, min_length = path.length();

    if ((len = tmp.length()) < min_length) {
      min_length = len;
      path = tmp;
    }
    tmp = dubinsRSL(d, alpha, beta);
    if ((len = tmp.length()) < min_length) {
      min_length = len;
      path = tmp;
    }
    tmp = dubinsLSR(d, alpha, beta);
    if ((len = tmp.length()) < min_length) {
      min_length = len;
      path = tmp;
    }
    tmp = dubinsRLR(d, alpha, beta);
    if ((len = tmp.length()) < min_length) {
      min_length = len;
      path = tmp;
    }
    tmp = dubinsLRL(d, alpha, beta);
    if ((len = tmp.length()) < min_length) {
      path = tmp;
    }
    std::vector<Segment> result;
    for( int i=0; i<3; i++ ) {
      double curvature = 0;
      switch(path.type_[i]) {
        case DUBINS_LEFT:
          curvature = 1;
          break;
        case DUBINS_RIGHT:
          curvature = -1;
          break;
        case DUBINS_STRAIGHT:
          curvature = 0;
          break;
      }
      result.push_back(Segment(path.length_[i], curvature));
    }
    return result;
  }

  std::vector<Segment> dubins_path(double radius,
      double x, double y, double theta) {
    // scale input to a radius of 1
    std::vector<Segment> raw = dubins_path(x/radius, y/radius, theta);
    std::vector<Segment> result;
    // scale result by radius
    for( int i=0; i<raw.size(); i++ ) {
      result.push_back(Segment(raw[i].getLength() * radius,
          raw[i].getCurvature() / radius));
    }
    return result;
  }

  std::vector<Segment> dubins_path(double radius,
      double x1, double y1, double theta1,
      double x2, double y2, double theta2) {
    // normalize and call dubins_path(r, x, y, t)

    // tanslate to the origin
    double x = x2 - x1;
    double y = y2 - y1;

    // compute distance and direction
    double d = sqrt(x*x + y*y);
    double th = atan2(y, x);

    // rotate by -theta1
    double theta = theta2 - theta1;
    th -= theta1;
    x = d * cos(th);
    y = d * sin(th);
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

